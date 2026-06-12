pub use axdma::{DMAInfo, alloc_coherent, dealloc_coherent};
use bitfield_struct::bitfield;

//DMA buffer,学长的代码引用了axdma中的DMAinfo来处理数据缓冲区的地址,我这里还没有引入
pub struct DMABuffer {
    pub addr: DMAInfo,
    pub size: usize,
}
//然后是有关DMA描述符的内容,这里主要设计了四个结构体来构建描述符链表
#[repr(C, align(4))]
pub struct IdmacDescriptor {
    //首先设置的控制描述符,应用于描述符的所有权
    pub des0: IdmacDes0,
    //然后是数据的长度
    pub des1: IdmacDes1,
    //这个用来放第一个buffer的物理地址
    pub des2: u32,
    //存放下一个描述符的物理地址
    pub des3: u32,
}
#[bitfield(u32, order = Msb)]
//des0的具体内容

pub struct IdmacDes0 {
    //首先设置一个位来决定当前描述符的所有权,因为是Msb,所以是第31位,1归IDMAC,0归CPU
    pub ownership: bool,
    //然后又设置一个错误位,当传输过程中出现错误,该位会被自动设置为1,具体的错误位在RINTSTS（原始中断状态寄存器）中,当错误发生,需要去那里找错误原因
    pub ces: bool,
    #[bits(24)]
    //占位符
    pub _reserved1: u32,
    //环形结束位（第 5 位）。置 1 时，表示描述符列表到达了最后。IDMAC 将返回列表的基地址，形成一个描述符环。这通常只在双缓冲结构中有意义
    pub er: bool,
    //然后设置了一个位控制DMA的传输方式,置 1 时，表示描述符中的第二个地址（也就是 des3）是下一个描述符的地址，而不是第二个数据缓冲区的地址
    pub ch: bool,
    //然后是fs和ld,分别代表所指数据是第一个或者最后一个缓冲区
    pub fs: bool,
    pub ld: bool,
    //然后是一个免中断打扰位
    pub dic: bool,

    #[bits(1)]
    pub _reserved0: u8,
}

#[bitfield(u32, order = Msb)]
//des1是数据长度
pub struct IdmacDes1 {
    #[bits(19)]
    //占位符
    pub _reserved1: u32,
    //缓冲区的大小
    #[bits(13)]
    pub bs1: u16,
}
impl IdmacDescriptor {
    //创建新描述符
    pub fn new() -> Self {
        Self {
            des0: IdmacDes0::default(),
            des1: IdmacDes1::default(),
            des2: 0,
            des3: 0,
        }
    }
    //设置des0
    pub fn set_desc0_control_descriptor(
        &mut self,
        own: bool,
        ces: bool,
        er: bool,
        ch: bool,
        fs: bool,
        ld: bool,
        dic: bool,
    ) {
        self.des0 = IdmacDes0::new()
            .with_ownership(own)
            .with_ces(ces)
            .with_er(er)
            .with_ch(ch)
            .with_fs(fs)
            .with_ld(ld)
            .with_dic(dic);
    }
    //设置des1
    pub fn set_des1_buffer1_size(&mut self, size: u16) {
        self.des1.set_bs1(size);
    }
    //设置des2
    pub fn set_des2_buffer1_address(&mut self, addr: u32) {
        self.des2 = addr;
    }
    //设置des3
    pub fn set_des3_next_descriptor_address(&mut self, addr: u32) {
        self.des3 = addr;
    }
}
//接下来想做的是支持描述符环，从而允许在没有 CPU 干预的情况下进行多块数据传输
//可以尝试
