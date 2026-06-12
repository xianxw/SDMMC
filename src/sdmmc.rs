use core::ptr::NonNull;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering, fence};
use core::time::Duration;

use axtask::WaitQueue;
use log::{debug, info, trace, warn};
use volatile::VolatilePtr;

use crate::{
    cmd::{Command, DataXfer},
    dma::DMABuffer,
    regs::{ClkDiv, ClkEna, RegisterBlock, RegisterBlockVolatileFieldAccess},
    utils::{Cid, CsdV2},
};

/// DMA 传输完成原子标志
pub static IDMAC_DONE_FLAG: AtomicBool = AtomicBool::new(false);

/// DMA 传输错误原子标志
pub static IDMAC_ERROR_FLAG: AtomicBool = AtomicBool::new(false);

/// DMA 传输等待队列
pub static IDMAC_WAIT_QUEUE: WaitQueue = WaitQueue::new();

/// SD/MMC 控制器寄存器基地址
pub static SDMMC_BASE_ADDR: AtomicUsize = AtomicUsize::new(0);

/// AHB总线数据位宽
#[derive(Debug, Clone, Copy)]
pub enum AHBDataWidth {
    /// 16位宽度
    Bits16,
    /// 32位宽度
    Bits32,
    /// 64位宽度
    Bits64,
}

impl AHBDataWidth {
    // Returns the alignment requirement in bytes for the given data width.
    pub fn align_value(&self) -> usize {
        match self {
            AHBDataWidth::Bits16 => 2,
            AHBDataWidth::Bits32 => 4,
            AHBDataWidth::Bits64 => 8,
        }
    }
}

fn wait_until<F>(mut f: F)
where
    F: FnMut() -> bool,
{
    // TODO: yield?
    while !f() {
        core::hint::spin_loop();
    }
}

/// SD/MMC driver.
///由于引入dma传输,所以显然要再加一个有关dma缓冲区的部分
pub struct SdMmc {
    //寄存器的控制
    regs: VolatilePtr<'static, RegisterBlock>,
    //块数目
    num_blocks: u64,
    //缓冲区
    dma_buffer: Option<DMABuffer>,
    //AHB总线数据位宽
    ahb_data_width: AHBDataWidth,
}

impl SdMmc {
    const FIFO: usize = 0x200;

    /// Creates a new `SdMmc` instance from the given base address.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `base` is a valid pointer to the SD/MMC controller's
    /// register block and that no other code is concurrently accessing the same hardware.
    pub unsafe fn new(
        base: usize,
        ahb_data_width: AHBDataWidth,
        register_irq: impl FnOnce() -> bool,
    ) -> Self {
        SDMMC_BASE_ADDR.store(base, Ordering::Release);
        let regs = unsafe { VolatilePtr::new(NonNull::new_unchecked(base as *mut _)) };

        let mut this = Self {
            regs,
            num_blocks: 0,
            dma_buffer: None,
            ahb_data_width,
        };
        this.init();
        let _ = this.try_enable_idmac(register_irq);
        this
    }

    fn can_send_cmd(&self) -> bool {
        !self.regs.cmd().read().start_cmd()
    }

    fn can_send_data(&self) -> bool {
        !self.regs.status().read().data_busy()
    }

    fn has_response(&self) -> bool {
        self.regs.rintsts().read().command_done()
    }

    fn fifo_cnt(&self) -> usize {
        self.regs.status().read().fifo_count() as usize
    }

    fn set_transaction_size(&self, blk_size: u16, byte_cnt: u32) {
        self.regs.blksiz().update(|r| r.with_block_size(blk_size));
        self.regs.bytcnt().write(byte_cnt);
    }

    /// 清空 DMA (IDMAC) 的所有中断和状态标志
    fn clear_idmac_interrupts(&self) {
        let idsts = self.regs.idsts().read();
        self.regs.idsts().write(idsts);
    }
    /// 发送 SD/MMC 命令，并等待其响应和数据传输（如果是 PIO 模式）。
    pub fn send_cmd(&self, command: Command<'_>) -> Option<[u32; 4]> {
        trace!("send_cmd {command:#x?}");
        let is_go_idle = matches!(command, Command::GoIdleState);
        let is_reset_clock = matches!(command, Command::ResetClock);
        let (cmd, arg, xfer) = command.build();
        assert_eq!(cmd.data_expected(), xfer.is_some());

        trace!("send_cmd {cmd:?} {arg:#x?}");
        //不采用wait_until 防止死循环,最多循环1000000次
        let mut cmd_wait_count = 0u64;
        let cmd_max_wait = 1_000_000u64; //1M最大循环
        while !self.can_send_cmd() {
            core::hint::spin_loop();
            cmd_wait_count += 1;
            if cmd_wait_count > cmd_max_wait {
                if is_go_idle {
                    warn!(
                        "    can_send_cmd timeout after {} iterations",
                        cmd_wait_count
                    );
                }
                break;
            }
        }
        if cmd.data_expected() {
            let mut data_wait_count = 0u64;
            while !self.can_send_data() {
                core::hint::spin_loop();
                data_wait_count += 1;
            }
            if data_wait_count > 1000 && is_reset_clock {
                info!(
                    "    can_send_data: true (waited {} iterations)",
                    data_wait_count
                );
            }
        }

        self.regs.cmdarg().write(arg);
        self.regs.cmd().write(cmd);

        let mut start_cmd_wait_count = 0u64;
        while !self.can_send_cmd() {
            core::hint::spin_loop();
            start_cmd_wait_count += 1;
            if start_cmd_wait_count > cmd_max_wait {
                if is_go_idle {
                    warn!(
                        "    start_cmd clear timeout after {} iterations",
                        start_cmd_wait_count
                    );
                }
                break;
            }
        }
        trace!("cmd {} sent", cmd.cmd_index());

        if cmd.response_expect() {
            let mut resp_wait_count = 0u64;
            while !self.has_response() {
                core::hint::spin_loop();
                resp_wait_count += 1;
                if resp_wait_count > cmd_max_wait {
                    if is_go_idle {
                        warn!("    response timeout after {} iterations", resp_wait_count);
                        let status_timeout = self.regs.status().read();
                        let rintsts_timeout = self.regs.rintsts().read();
                        warn!("    Status at timeout: {:?}", status_timeout);
                        warn!("    RINTSTS at timeout: {:?}", rintsts_timeout);
                    }
                    break;
                }
            }
            trace!("cmd {} received response", cmd.cmd_index());
        }

        if let Some(xfer) = xfer {
            let fifo_base = unsafe { self.regs.as_raw_ptr().byte_add(Self::FIFO) }.cast::<u64>();
            let mut offset = 0;
            match xfer {
                DataXfer::Read(buf) => {
                    assert_eq!(buf.len() % 8, 0);
                    wait_until(|| {
                        let rintsts = self.regs.rintsts().read();

                        if rintsts.receive_fifo_data_request() {
                            while self.fifo_cnt() >= 2 && offset < buf.len() {
                                let data = unsafe { fifo_base.read_volatile() };
                                buf[offset..offset + 8].copy_from_slice(&data.to_le_bytes());
                                offset += 8;
                            }
                        }

                        rintsts.data_transfer_over() || rintsts.error() || offset >= buf.len()
                    });
                    trace!("received {offset} bytes");
                }
                DataXfer::Write(buf) => {
                    assert_eq!(buf.len() % 8, 0);
                    wait_until(|| {
                        let rintsts = self.regs.rintsts().read();

                        if rintsts.transmit_fifo_data_request() {
                            while self.fifo_cnt() < 120 && offset < buf.len() {
                                let data =
                                    u64::from_le_bytes(buf[offset..offset + 8].try_into().unwrap());
                                unsafe { fifo_base.write_volatile(data) };
                                offset += 8;
                            }
                        }

                        rintsts.data_transfer_over() || rintsts.error()
                    });
                    trace!("sent {offset} bytes");
                }
            }
        }

        let resp = self.regs.resp().read();

        let rintsts = self.regs.rintsts().read();
        // clear interrupt status
        self.regs.rintsts().write(rintsts);

        if rintsts.error() {
            return None;
        }
        Some(resp)
    }

    /// 初始化 SD/MMC 控制器和底层的 SD 卡。
    pub fn init(&mut self) {
        info!("Initializing SD/MMC driver at {:?}", self.regs);

        // On VisionFive2, some registers have been initialized by the bootloader(U-Boot).
        // But some default values are not suitable for our driver, so we need to reset and reconfigure them.
        trace!("ctrl: {:?}", self.regs.ctrl().read());
        trace!("pwren: {:?}", self.regs.pwren().read());
        trace!("clkdiv: {:?}", self.regs.clkdiv().read());
        trace!("clksrc: {:?}", self.regs.clksrc().read());
        trace!("clkena: {:?}", self.regs.clkena().read());
        trace!("tmout: {:?}", self.regs.tmout().read());
        trace!("ctype: {:?}", self.regs.ctype().read());
        trace!("cdetect: {:?}", self.regs.cdetect().read());
        trace!("wrtprt: {:?}", self.regs.wrtprt().read());
        trace!("usrid: {:?}", self.regs.usrid().read());
        trace!("verid: {:?}", self.regs.verid().read());
        trace!("hcon: {:?}", self.regs.hcon().read());
        trace!("uhs: {:?}", self.regs.uhs().read());
        trace!("bmod: {:?}", self.regs.bmod().read());
        trace!("dbaddr: {:?}", self.regs.dbaddr().read());

        // Clear any stale interrupt status flags left by bootloader
        // Writing 1 to these bits clears them
        let rintsts = self.regs.rintsts().read();
        trace!("initial rintsts: {rintsts:?}");
        self.regs.rintsts().write(rintsts);
        trace!("cleared interrupt status");

        // Disable clock for configuration
        self.regs.clkena().write(ClkEna::new());
        let _ = self.send_cmd(Command::ResetClock);

        // Set clock divider to lower frequency (slower for compatibility)
        self.regs
            .clkdiv()
            .write(ClkDiv::new().with_clk_divider0(100));

        // Now enable clock with new divider
        self.regs.clkena().write(ClkEna::new().with_cclk_enable(1));
        let _ = self.send_cmd(Command::ResetClock);

        // Long delay to let everything stabilize
        for _ in 0..10000 {
            core::hint::spin_loop();
        }

        // Enable card power if available in PWREN register
        self.regs.pwren().write(1u32.into()); // Card power enable

        // Increased stabilization delay
        for _ in 0..100000 {
            core::hint::spin_loop();
        }

        // set data width -> 1bit
        self.regs.ctype().write(0.into());

        // reset dma
        self.regs.bmod().update(|r| r.with_de(false).with_swr(true));
        self.regs
            .ctrl()
            .update(|r| r.with_dma_reset(true).with_use_internal_dmac(false));

        trace!("dma reset");

        // Note: GoIdleState may timeout during initial card detection phase.
        // This is not fatal - the card responds to SendIfCond and continues initialization normally.
        let _ = self.send_cmd(Command::GoIdleState);
        trace!("idle state set");

        let has_valid_resp = match self.send_cmd(Command::SendIfCond(0x1aa)) {
            Some(resp) => resp[0] & 0xff == 0xaa,
            None => false,
        };

        if !has_valid_resp {
            debug!("SD card not responding properly to SendIfCond - continuing anyway");
        }

        let mut attempt = 0;
        let mut card_initialized = false;
        loop {
            attempt += 1;
            if attempt > 100 {
                break;
            }

            if self.send_cmd(Command::AppCmd(0)).is_none() {
                continue;
            }

            if let Some(resp) = self.send_cmd(Command::SdSendOpCond(0x41FF_8000)) {
                let ocr = resp[0];
                if ocr & 0x8000_0000 != 0 {
                    card_initialized = true;
                    if ocr & 0x4000_0000 != 0 {
                        debug!("SD card supports high capacity");
                    } else {
                        debug!("SD card is standard capacity");
                    }
                    break;
                }
            }

            core::hint::spin_loop();
        }

        if !card_initialized {
            warn!("Card initialization failed - ACMD41 loop timed out");
            return;
        }

        let cid = match self.send_cmd(Command::AllSendCid) {
            Some(resp) => unsafe { core::mem::transmute::<[u32; 4], Cid>(resp) },
            None => {
                warn!("AllSendCid failed - cannot determine card ID");
                return;
            }
        };
        debug!("cid: {cid:?}");

        let rca = match self.send_cmd(Command::SendRelativeAddr) {
            Some(resp) => (resp[0] >> 16) & 0xffff,
            None => {
                warn!("SendRelativeAddr failed - cannot get card address");
                return;
            }
        };
        debug!("rca: {rca:#x}");

        match self.send_cmd(Command::SendCsd(rca << 16)) {
            Some(resp) => {
                let csd = unsafe { core::mem::transmute::<[u32; 4], CsdV2>(resp) };
                debug!("csd: {csd:?}");
                self.num_blocks = csd.num_blocks();
                info!("SD card capacity: {:#x} blocks", self.num_blocks);
            }
            None => {
                warn!("SendCsd failed - cannot determine card capacity");
                self.num_blocks = 0;
            }
        }

        if self.send_cmd(Command::SelectCard(rca << 16)).is_none() {
            warn!("SelectCard failed");
        }

        if self.send_cmd(Command::AppCmd(rca << 16)).is_none() {
            warn!("AppCmd failed");
        }

        // Read SCR register of SD card to determine supported bus widths.
        self.set_transaction_size(8, 8);
        let mut buf = [0u8; 512];
        if self.send_cmd(Command::SendScr(&mut buf)).is_none() {
            warn!("SendScr failed");
        }

        let resp = unsafe {
            self.regs
                .as_raw_ptr()
                .byte_add(Self::FIFO)
                .cast::<u64>()
                .read_volatile()
        };
        debug!("Bus width supported: {:#x?}", (resp >> 8) & 0xf);

        let rintsts = self.regs.rintsts().read();
        self.regs.rintsts().write(rintsts); // clear interrupt status

        info!("SD/MMC driver initialized");
    }
    /// 尝试为 SD/MMC 控制器启用内置 DMA 控制器 (IDMAC)。
    ///
    /// 该函数会执行以下步骤：
    /// 1. 分配对齐的物理连续 DMA 缓冲区。
    /// 2. 清除原始中断状态和 DMA 状态寄存器。
    /// 3. 配置总线模式（BMOD）和控制（CTRL）寄存器启用 DMA 模式，并进行写后读校验。
    /// 4. 在 IDINTEN 和 INTMASK 中使能 DMA 相关中断，并进行写后读校验。
    /// 5. 执行 `register_irq` 闭包注册硬件中断。
    /// 6. 如果中途任何一步失败，将自动释放内存并恢复至 PIO 模式以安全降级（包括对 BMOD 和 CTRL 的软硬件复位）。
    /// 如果成功启用则返回 `Ok(())`，否则返回包含错误信息的 `Err(&'static str)`。
    pub fn try_enable_idmac<F>(&mut self, register_irq: F) -> Result<(), &'static str>
    where
        F: FnOnce() -> bool,
    {
        // 1. 分配对齐的物理连续 DMA 缓冲区 (DMABuffer)
        let layout = core::alloc::Layout::from_size_align(
            Self::BLOCK_SIZE,
            self.ahb_data_width.align_value(),
        )
        .expect("Invalid layout for DMA buffer");

        let dma_info = match unsafe { crate::dma::alloc_coherent(layout) } {
            Ok(dma_info) => {
                info!(
                    "DMA buffer allocated: virt=0x{:08x}, phys=0x{:08x}, size={}",
                    dma_info.cpu_addr.as_ptr() as usize,
                    dma_info.bus_addr.as_u64(),
                    Self::BLOCK_SIZE
                );
                dma_info
            }
            Err(e) => {
                warn!(
                    "Failed to allocate DMA buffer: {:?}, use PIO mode instead",
                    e
                );
                return Err("Failed to allocate coherent DMA memory");
            }
        };

        // 后续配置逻辑，如果执行失败，会在 Err 分支中执行清理回退
        let configure_and_register = || {
            // 2. 清除残留状态 (RINTSTS & IDSTS)
            let rintsts = self.regs.rintsts().read();
            self.regs.rintsts().write(rintsts);

            let idsts = self.regs.idsts().read();
            self.regs.idsts().write(idsts);

            // 3. 配置 BMOD（总线模式）寄存器：de = true, fb = true
            self.regs.bmod().update(|r| r.with_de(true).with_fb(true));
            // 硬件校验: Read-after-Write
            let bmod_val = self.regs.bmod().read();
            if !bmod_val.de() || !bmod_val.fb() {
                return Err("Hardware verification failed: BMOD de/fb not set");
            }

            // 4. 配置 CTRL（控制）寄存器：use_internal_dmac = true, int_enable = true
            self.regs
                .ctrl()
                .update(|r| r.with_use_internal_dmac(true).with_int_enable(true));
            // 硬件校验: Read-after-Write
            let ctrl_val = self.regs.ctrl().read();
            if !ctrl_val.use_internal_dmac() || !ctrl_val.int_enable() {
                return Err(
                    "Hardware verification failed: CTRL use_internal_dmac/int_enable not set",
                );
            }

            // 5. 使能 DMA 相关中断
            use crate::regs::IdIntEn;
            let idinten = IdIntEn::new()
                .with_ni(true) // 正常中断汇总使能
                .with_ai(true) // 异常中断汇总使能
                .with_ti(true) // 发送完成中断使能
                .with_ri(true) // 接收完成中断使能
                .with_fbe(true) // 致命总线错误使能
                .with_du(true) // 描述符不可用使能
                .with_ces(true); // 卡错误汇总使能
            self.regs.idinten().write(idinten);
            // 硬件校验: Read-after-Write
            let idinten_val = self.regs.idinten().read();
            if !idinten_val.ni()
                || !idinten_val.ai()
                || !idinten_val.ti()
                || !idinten_val.ri()
                || !idinten_val.fbe()
                || !idinten_val.du()
                || !idinten_val.ces()
            {
                return Err("Hardware verification failed: IDINTEN flags not matching");
            }

            // 在 INTMASK 中允许数据传输结束 (dto) 中断
            self.regs.intmask().update(|r| r.with_dto(true));
            // 硬件校验: Read-after-Write
            let intmask_val = self.regs.intmask().read();
            if !intmask_val.dto() {
                return Err("Hardware verification failed: INTMASK dto not set");
            }

            // 6. 注册硬件中断处理函数 (IRQ)
            if !register_irq() {
                return Err("Failed to register IRQ");
            }

            Ok(())
        };

        match configure_and_register() {
            Ok(()) => {
                self.dma_buffer = Some(DMABuffer {
                    addr: dma_info,
                    size: Self::BLOCK_SIZE,
                });
                Ok(())
            }
            Err(err) => {
                // 7. 回退与清理机制：释放内存并恢复/复位寄存器状态
                unsafe {
                    crate::dma::dealloc_coherent(dma_info, layout);
                }

                // 强化硬件复位，防止硬件卡死
                self.regs.bmod().update(|r| r.with_de(false).with_swr(true));
                wait_until(|| !self.regs.bmod().read().swr());

                self.regs
                    .ctrl()
                    .update(|r| r.with_use_internal_dmac(false).with_dma_reset(true));
                wait_until(|| !self.regs.ctrl().read().dma_reset());

                self.regs.idinten().write(0.into());
                self.dma_buffer = None;
                Err(err)
            }
        }
    }

    ///实现dma传输
    pub fn send_cmd_idmac(&self, cmd: Command<'_>) -> Option<[u32; 4]> {
        let (cmd, arg, xfer) = cmd.build();
        assert!(
            cmd.data_expected(),
            "send_cmd_idmac should only be used for commands that require data transfer"
        );
        assert!(
            xfer.is_some(),
            "send_cmd_idmac requires a data buffer for transfer"
        );

        // 第一阶段：准备与状态清理，并保存基准硬件状态
        let baseline_idsts = self.regs.idsts().read();
        let baseline_rintsts = self.regs.rintsts().read();

        let rintsts = self.regs.rintsts().read();
        self.regs.rintsts().write(rintsts);
        self.clear_idmac_interrupts();

        // 解析缓冲区信息
        let (buf_ptr, buf_size, _) = match xfer {
            Some(DataXfer::Read(buf)) => (buf.as_mut_ptr(), buf.len(), false),
            Some(DataXfer::Write(buf)) => (buf.as_ptr() as *mut u8, buf.len(), true),
            None => unreachable!(),
        };

        // 第二阶段：构建并挂载 DMA 描述符
        use crate::dma::IdmacDescriptor;
        use core::alloc::Layout;

        let layout = Layout::new::<IdmacDescriptor>();
        let dma_desc_info = unsafe { crate::dma::alloc_coherent(layout) }
            .expect("Failed to allocate DMA descriptor");
        let desc_ptr = dma_desc_info.cpu_addr.as_ptr() as *mut IdmacDescriptor;

        let desc = unsafe { &mut *desc_ptr };
        *desc = IdmacDescriptor::new();
        desc.set_desc0_control_descriptor(
            true,  // ownership: 归 IDMAC 所有
            false, // ces: 无卡错误
            false, // er: 环形结束标记（单描述符）
            false, // ch: 链式描述符结构
            true,  // fs: 第一缓冲区
            true,  // ld: 最后一缓冲区
            false, // dic: 触发中断
        );
        desc.set_des1_buffer1_size(buf_size as u16);
        desc.set_des2_buffer1_address(buf_ptr as u32);
        desc.set_des3_next_descriptor_address(0);

        // 将描述符的物理/内存地址挂载到控制器的 dbaddr 寄存器中
        let desc_bus_addr = dma_desc_info.bus_addr.as_u64() as u32;
        fence(Ordering::Release);
        self.regs.dbaddr().write(desc_bus_addr);

        // 第三阶段：编程寄存器并激活 DMA
        let blk_size = if buf_size < 512 { buf_size as u16 } else { 512 };
        self.set_transaction_size(blk_size, buf_size as u32);

        // 在触发 DMA 之前重置标志位
        IDMAC_DONE_FLAG.store(false, Ordering::Release);
        IDMAC_ERROR_FLAG.store(false, Ordering::Release);

        // 发送 SD/MMC 命令
        wait_until(|| self.can_send_cmd());
        if cmd.data_expected() {
            wait_until(|| self.can_send_data());
        }
        self.regs.cmdarg().write(arg);
        self.regs.cmd().write(cmd);

        // 等待控制器硬件（CIU）成功接收并装载命令（即 start_cmd 被清空）
        wait_until(|| self.can_send_cmd());

        // 优化唤醒 DMA (pldmnd) 的时机：一旦硬件清空 start_cmd，立即写入 pldmnd 唤醒 DMA 传输
        self.regs.pldmnd().write(1);

        // 检查 DMA 描述符不可用状态（du 位），如果出现了 du，补发一次 pldmnd 防止 FIFO 下溢
        let current_idsts = self.regs.idsts().read();
        if current_idsts.du() {
            self.regs
                .idsts()
                .write(crate::regs::IdSts::new().with_du(true));
            self.regs.pldmnd().write(1);
        }

        // 第一阶段：等待中断通知 (IDMAC_DONE_FLAG)
        let deadline = axhal::time::wall_time() + Duration::from_secs(1);
        let mut dma_irq_timed_out = false;
        while !IDMAC_DONE_FLAG.load(Ordering::Acquire) {
            if axhal::time::wall_time() >= deadline {
                dma_irq_timed_out = true;
                break;
            }
            axtask::yield_now();
        }

        let rintsts_during_irq = self.regs.rintsts().read();
        let idsts_during_irq = self.regs.idsts().read();
        if dma_irq_timed_out {
            warn!("send_cmd_idmac: DMA IRQ did not arrive within 1 second");
            warn!(
                "send_cmd_idmac: timeout rintsts={rintsts_during_irq:?} idsts={idsts_during_irq:?}"
            );
            warn!(
                "send_cmd_idmac: DMA transfer appears stalled, check IDMAC/SDMMC interrupt enable and descriptor status"
            );
        } else {
            info!(
                "send_cmd_idmac: DMA IRQ received; rintsts={rintsts_during_irq:?}, idsts={idsts_during_irq:?}"
            );
        }

        // 第二阶段：等待命令发送与响应接收，检查命令阶段错误
        if cmd.response_expect() {
            debug!("send_cmd_idmac: waiting for command response");
            let response_deadline = axhal::time::wall_time() + Duration::from_secs(2);
            let mut response_wait_count = 0u64;
            let response_wait_log_interval = 1_000_000u64;
            let mut response_timeout = false;
            let mut command_error = false;

            while !self.has_response() {
                core::hint::spin_loop();
                response_wait_count += 1;

                let rintsts = self.regs.rintsts().read();
                if rintsts.error() {
                    command_error = true;
                    warn!(
                        "send_cmd_idmac: command error detected! rintsts={:?}",
                        rintsts
                    );
                    break;
                }

                if response_wait_count % response_wait_log_interval == 0 {
                    warn!(
                        "send_cmd_idmac: waiting for response after {} iterations; rintsts={:?}",
                        response_wait_count, rintsts
                    );
                }

                if axhal::time::wall_time() >= response_deadline {
                    response_timeout = true;
                    warn!(
                        "send_cmd_idmac: response timeout after {} iterations, cmd {}",
                        response_wait_count,
                        cmd.cmd_index()
                    );
                    break;
                }
            }

            if response_timeout || command_error {
                let rintsts = self.regs.rintsts().read();
                warn!(
                    "send_cmd_idmac: command response failed for cmd {}; rintsts={:?}",
                    cmd.cmd_index(),
                    rintsts
                );
                self.regs.rintsts().write(rintsts);
                self.clear_idmac_interrupts();
                unsafe {
                    crate::dma::dealloc_coherent(dma_desc_info, layout);
                }
                return None; // 提前返回并释放描述符内存，避免内存泄漏
            }

            debug!(
                "send_cmd_idmac: command response received after {} iterations",
                response_wait_count
            );
        } else {
            // 如果不需要响应，仅检查命令发送是否有即时错误
            let rintsts = self.regs.rintsts().read();
            if rintsts.error() {
                warn!(
                    "send_cmd_idmac: command phase error for cmd {}; rintsts={:?}",
                    cmd.cmd_index(),
                    rintsts
                );
                self.regs.rintsts().write(rintsts);
                self.clear_idmac_interrupts();
                unsafe {
                    crate::dma::dealloc_coherent(dma_desc_info, layout);
                }
                return None;
            }
        }

        // 第三阶段：等待数据传输结束，并进行防御性状态轮询
        let mut last_status = None;
        let data_deadline = axhal::time::wall_time() + Duration::from_secs(5);
        let mut data_wait_count = 0u64;
        let mut data_timeout = false;
        let mut polling_error = false;

        while data_wait_count < u64::MAX {
            if axhal::time::wall_time() >= data_deadline {
                data_timeout = true;
                warn!(
                    "send_cmd_idmac: data_transfer_over timeout after {} iterations for cmd {}",
                    data_wait_count,
                    cmd.cmd_index()
                );
                break;
            }

            let rintsts = self.regs.rintsts().read();
            let idsts = self.regs.idsts().read();

            // 防御性状态轮询：对比基准状态以检测新发生的致命错误
            let new_fbe = idsts.fbe() && !baseline_idsts.fbe();
            let new_ces = idsts.ces() && !baseline_idsts.ces();
            let new_du = idsts.du() && !baseline_idsts.du();
            let new_rint_error = rintsts.error() && !baseline_rintsts.error();

            last_status = Some((
                rintsts,
                idsts,
                new_fbe || new_ces || new_du || new_rint_error,
            ));

            if new_fbe || new_ces || new_du || new_rint_error {
                warn!(
                    "send_cmd_idmac: Defensive check detected new fatal error during data phase! baseline_idsts={:?}, current_idsts={:?}, baseline_rintsts={:?}, current_rintsts={:?}",
                    baseline_idsts, idsts, baseline_rintsts, rintsts
                );
                polling_error = true;
                break;
            }

            // 当 DMA 中断已标记完成，或控制器报告数据传输结束，或发生控制器错误时退出循环
            if IDMAC_DONE_FLAG.load(Ordering::Acquire)
                || rintsts.data_transfer_over()
                || rintsts.error()
            {
                break;
            }

            data_wait_count += 1;
            core::hint::spin_loop();
        }

        // 读取响应并校验错误
        let resp = self.regs.resp().read();

        let (rintsts, idsts, idmac_new_error) = last_status.unwrap_or_else(|| {
            let rintsts = self.regs.rintsts().read();
            let idsts = self.regs.idsts().read();
            (rintsts, idsts, false)
        });

        debug!(
            "send_cmd_idmac final wait result: rintsts={:?}, idsts={:?}, idmac_new_error={}",
            rintsts, idsts, idmac_new_error,
        );

        // 清理状态
        self.clear_idmac_interrupts();
        self.regs.rintsts().write(rintsts);

        // 释放描述符一致性内存
        unsafe {
            crate::dma::dealloc_coherent(dma_desc_info, layout);
        }

        // 校验是否有错误发生
        let has_error = IDMAC_ERROR_FLAG.load(Ordering::Acquire)
            || dma_irq_timed_out
            || data_timeout
            || polling_error
            || rintsts.error()
            || idmac_new_error;

        if has_error {
            trace!("IDMAC transfer failed! has_error flagged.");
            None
        } else {
            info!(
                "send_cmd_idmac: transfer complete for cmd {}; resp={:?}",
                cmd.cmd_index(),
                resp
            );
            Some(resp)
        }
    }

    /// Reads a single block from the SD/MMC card.
    pub fn read_block(&mut self, block: u32, buf: &mut [u8; 512]) {
        self.set_transaction_size(512, 512);

        info!("read block: block={}", block);

        if let Some(dma_buf_info) = &self.dma_buffer {
            trace!(
                "Using DMA buffer for read: virt=0x{:08x}, phys=0x{:08x}, size={}",
                dma_buf_info.addr.cpu_addr.as_ptr() as usize,
                dma_buf_info.addr.bus_addr.as_u64() as usize,
                dma_buf_info.size
            );

            let dma_buf_phy_ptr = dma_buf_info.addr.bus_addr.as_u64() as *mut u8;
            let dma_buf = unsafe { core::slice::from_raw_parts_mut(dma_buf_phy_ptr, buf.len()) };

            info!(
                "read_block: before send_cmd_idmac - BlkSiz={:?}, ByteCnt=0x{:08x}, CType={:?}, FIFOTH={:?}",
                self.regs.blksiz().read(),
                self.regs.bytcnt().read(),
                self.regs.ctype().read(),
                self.regs.fifoth().read(),
            );
            self.send_cmd_idmac(Command::ReadSingleBlock(block, dma_buf))
                .unwrap();
            info!(
                "read_block: after send_cmd_idmac - BlkSiz={:?}, ByteCnt=0x{:08x}, CType={:?}, FIFOTH={:?}",
                self.regs.blksiz().read(),
                self.regs.bytcnt().read(),
                self.regs.ctype().read(),
                self.regs.fifoth().read(),
            );

            let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
            let dma_usr_slice = unsafe { core::slice::from_raw_parts(dma_buf_virt_ptr, buf.len()) };
            buf.copy_from_slice(dma_usr_slice);
        } else {
            warn!("No DMA buffer available - read may fail or be very slow");
            self.send_cmd(Command::ReadSingleBlock(block, buf)).unwrap();
        }

        trace!("fifo count: {}", self.fifo_cnt());
    }

    /// Writes a single block to the SD/MMC card.
    pub fn write_block(&mut self, block: u32, buf: &[u8; 512]) {
        self.set_transaction_size(512, 512);

        if let Some(dma_buf_info) = &self.dma_buffer {
            trace!(
                "Using DMA buffer for write: virt=0x{:08x}, phys=0x{:08x}, size={}",
                dma_buf_info.addr.cpu_addr.as_ptr() as usize,
                dma_buf_info.addr.bus_addr.as_u64() as usize,
                dma_buf_info.size
            );

            let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
            let dma_usr_slice =
                unsafe { core::slice::from_raw_parts_mut(dma_buf_virt_ptr, buf.len()) };
            dma_usr_slice.copy_from_slice(buf);

            let dma_buf_phy_ptr = dma_buf_info.addr.bus_addr.as_u64() as *mut u8;
            let dma_slice = unsafe { core::slice::from_raw_parts_mut(dma_buf_phy_ptr, buf.len()) };
            self.send_cmd_idmac(Command::WriteSingleBlock(block, dma_slice))
                .unwrap();
        } else {
            warn!("No DMA buffer available - write may fail or be very slow");
            self.send_cmd(Command::WriteSingleBlock(block, buf))
                .unwrap();
        }

        trace!("fifo count: {}", self.fifo_cnt());
    }

    /// Returns the number of blocks.
    pub fn num_blocks(&self) -> u64 {
        self.num_blocks
    }
    /// 中断处理函数
    ///
    /// 在真正的硬件中断触发时由操作系统的中断处理例程（ISR）调用。
    /// 该函数会读取控制器原始中断状态（RINTSTS）和 DMA 内部中断状态（IDSTS），
    /// 根据相应的状态进行错误处理、数据传输完成判断，并清空相关的中断标志。
    ///
    /// 本函数不接受 `self` 参数，通过全局的寄存器基地址 `SDMMC_BASE_ADDR` 访问硬件寄存器，
    /// 从而可以直接挂载到系统的中断向量表中。
    pub fn handle_interrupt() {
        let previous_flag = IDMAC_DONE_FLAG.load(Ordering::Acquire);
        debug!(
            "SdMmc::handle_interrupt entered; previous IDMAC_DONE_FLAG={}",
            previous_flag
        );
        let base = SDMMC_BASE_ADDR.load(Ordering::Acquire);
        let mut should_wake = false;

        if base != 0 {
            let regs =
                unsafe { VolatilePtr::new(NonNull::new_unchecked(base as *mut RegisterBlock)) };
            let rintsts = regs.rintsts().read();
            let idsts = regs.idsts().read();

            let has_rintsts = rintsts.sdio() != 0
                || rintsts.end_bit_error()
                || rintsts.auto_command_done()
                || rintsts.start_bit_error()
                || rintsts.hardware_locked_write()
                || rintsts.fifo_under_over_run()
                || rintsts.host_timeout()
                || rintsts.data_read_timeout()
                || rintsts.response_timeout()
                || rintsts.data_crc_error()
                || rintsts.response_crc_error()
                || rintsts.receive_fifo_data_request()
                || rintsts.transmit_fifo_data_request()
                || rintsts.data_transfer_over()
                || rintsts.command_done()
                || rintsts.response_error()
                || rintsts.card_detect();

            let has_idsts = idsts.ais()
                || idsts.nis()
                || idsts.ces()
                || idsts.du()
                || idsts.fbe()
                || idsts.ri()
                || idsts.ti();

            if has_idsts {
                debug!(
                    "SdMmc::handle_interrupt: clearing IDSTS in interrupt handler: {:?}",
                    idsts
                );
                if idsts.fbe() || idsts.du() || idsts.ces() {
                    log::error!("SDMMC DMA Error in interrupt! IDSTS: {:?}", idsts);
                    IDMAC_ERROR_FLAG.store(true, Ordering::Release);
                }
                regs.idsts().write(idsts);
                should_wake = true;
            }

            if rintsts.error() {
                log::error!(
                    "SDMMC Controller Error in interrupt! RINTSTS: {:?}",
                    rintsts
                );
                IDMAC_ERROR_FLAG.store(true, Ordering::Release);
                should_wake = true;
            }

            if rintsts.data_transfer_over()
                || rintsts.receive_fifo_data_request()
                || rintsts.transmit_fifo_data_request()
            {
                let mut clear_rintsts = crate::regs::RIntSts::new();
                clear_rintsts = clear_rintsts
                    .with_data_transfer_over(rintsts.data_transfer_over())
                    .with_receive_fifo_data_request(rintsts.receive_fifo_data_request())
                    .with_transmit_fifo_data_request(rintsts.transmit_fifo_data_request());
                debug!(
                    "SdMmc::handle_interrupt: clearing DTO/RXDR/TXDR bits in RINTSTS: {:?}",
                    clear_rintsts
                );
                regs.rintsts().write(clear_rintsts);
                should_wake = true;
            }

            if !has_rintsts && !has_idsts {
                warn!("SdMmc::handle_interrupt: IRQ entered with no RINTSTS/IDSTS bits set");
                warn!(
                    "SdMmc::handle_interrupt: stray IRQ? RINTSTS={:?} IDSTS={:?}",
                    rintsts, idsts
                );
            }
        } else {
            warn!("SdMmc::handle_interrupt: no SDMMC register base available to clear IDSTS");
        }

        if should_wake {
            IDMAC_DONE_FLAG.store(true, Ordering::Release);
            let after_flag = IDMAC_DONE_FLAG.load(Ordering::Acquire);
            debug!(
                "SdMmc::handle_interrupt: IDMAC_DONE_FLAG updated to {}",
                after_flag
            );
            IDMAC_WAIT_QUEUE.notify_one(true);
            debug!("SdMmc::handle_interrupt: notified wait queue");
        }
    }

    /// The size of a block in bytes.
    pub const BLOCK_SIZE: usize = 512;
}

unsafe impl Send for SdMmc {}
unsafe impl Sync for SdMmc {}

impl Drop for SdMmc {
    fn drop(&mut self) {
        if let Some(dma_buf) = self.dma_buffer.take() {
            if let Ok(layout) = core::alloc::Layout::from_size_align(
                dma_buf.size,
                self.ahb_data_width.align_value(),
            ) {
                unsafe {
                    crate::dma::dealloc_coherent(dma_buf.addr, layout);
                }
            }
        }
    }
}

// 当前中断处理函数尚未完善,send_cmd_idmac最终通过轮询的方式来通知cpu,且当前仍然未实现判断是否使用dma传输的函数
