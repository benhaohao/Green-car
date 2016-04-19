
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/delay.h>
//#include <mach/regs-gpio.h>
#include <linux/gpio.h>
//#include <plat/gpio-cfg.h>


//#define LCD_RESET IMX_GPIO_NR(3,16)
//#define LCD_DCX  IMX_GPIO_NR(3,17)
//#define SPI_CS  S3C2410_GPC(8)
//#define SPI_CLK S3C2410_GPD(0)
//#define SPI_SDI	S3C2410_GPC(9)
//#define RESET	S3C2410_GPD(1)

//#define SPI_CS  IMX_GPIO_NR(3,16)
//#define SPI_CLK IMX_GPIO_NR(3,17)
//#define SPI_SDI	IMX_GPIO_NR(3,18)
//#define RESET	IMX_GPIO_NR(3,19)

#define SPI_CS  IMX_GPIO_NR(5,29)
#define SPI_CLK IMX_GPIO_NR(2,23)
#define SPI_SDI	IMX_GPIO_NR(2,24)
#define RESET	IMX_GPIO_NR(2,25)

static void set_cs(int cs)
{
	gpio_set_value(SPI_CS,cs);
}

static void set_clk(int clk)
{
	gpio_set_value(SPI_CLK,clk);
}

static void set_outdata(int outdata)
{
	gpio_set_value(SPI_SDI,outdata);
}

static void send_data(unsigned int data)
{
	unsigned int i;
	int clk_num;
	int clk_mask;
	set_cs(0);
	udelay(10);
	set_clk(1);
	set_outdata(1);
	udelay(10);

	clk_num = 9;
	clk_mask = 0x100;
	udelay(50);

	for(i = 0; i < clk_num; ++i){
		set_clk(0);
		if(data & clk_mask){
			set_outdata(1);
		}
		else{
			set_outdata(0);
		}
		udelay(100);
		set_clk(1);
		udelay(100);
		data <<= 1;
	}
	set_outdata(1);
	set_cs(1);
}

static void st7789v_write_cmd(unsigned char cmd)
{
	unsigned int out;
	out = (cmd & 0xFF);
	send_data(out);
}

static void st7789v_write_data(unsigned char data)
{
	unsigned int out = (data & 0xFF) | 0x100;
	send_data(out);
}

static void set_st7789v(void)
{
	
	//--------------------------------ST7789S Frame rate setting----------------------------------//
	st7789v_write_cmd(0x3A);
//	st7789v_write_data(0x06);
	st7789v_write_data(0x05);
	//
	st7789v_write_cmd(0x36);
	st7789v_write_data(0x00);
	//--------------------------------ST7789S Frame rate setting----------------------------------//
	st7789v_write_cmd(0xb2);
	st7789v_write_data(0x0C);
	st7789v_write_data(0x0C);
	st7789v_write_data(0x00);
	st7789v_write_data(0x33);
	st7789v_write_data(0x33);


	st7789v_write_cmd(0xb7);
	st7789v_write_data(0x35);


	st7789v_write_cmd(0xbb);
	st7789v_write_data(0x28);//vcom

	st7789v_write_cmd(0xc0);
	st7789v_write_data(0x2C);

	st7789v_write_cmd(0xc2);
	st7789v_write_data(0x01);//17调深浅

	st7789v_write_cmd(0xc3);
	st7789v_write_data(0x10);

	st7789v_write_cmd(0xc4);
	st7789v_write_data(0x20);

	st7789v_write_cmd(0xc6);
	st7789v_write_data(0x0F);
	

	st7789v_write_cmd(0xd0);
	st7789v_write_data(0xa4);
	st7789v_write_data(0xa1);


	//--------------------------------ST7789S gamma setting---------------------------------------//
	st7789v_write_cmd(0xe0);
	st7789v_write_data(0xd0);
	st7789v_write_data(0x00);
	st7789v_write_data(0x02);
	st7789v_write_data(0x07);
	st7789v_write_data(0x0A);
	st7789v_write_data(0x28);
	st7789v_write_data(0x32);
	st7789v_write_data(0x44);
	st7789v_write_data(0x42);
	st7789v_write_data(0x06);
	st7789v_write_data(0x0e);
	st7789v_write_data(0x12);
	st7789v_write_data(0x14);
	st7789v_write_data(0x17);

	st7789v_write_cmd(0xe1);
	st7789v_write_data(0xd0);
	st7789v_write_data(0x00);
	st7789v_write_data(0x02);
	st7789v_write_data(0x07);
	st7789v_write_data(0x0A);
	st7789v_write_data(0x28);
	st7789v_write_data(0x31);
	st7789v_write_data(0x54);
	st7789v_write_data(0x47);
	st7789v_write_data(0x0E);
	st7789v_write_data(0x1C);
	st7789v_write_data(0x17);
	st7789v_write_data(0x1B);
	st7789v_write_data(0x1E);

	st7789v_write_cmd(0x21); //反显
	//st7789v_write_cmd(0x20); //反显
	
	//*********SET RGB Interfae***************
	st7789v_write_cmd(0xB1);
        st7789v_write_data(0xC2); // set DE mode ; SET Hs,Vs,DE,DOTCLK signal polarity 
	
        //st7789v_write_data(0x60); // set DE mode ; SET Hs,Vs,DE,DOTCLK signal polarity 
        //st7789v_write_data(0x00); 
        //st7789v_write_data(0x00); 
        //VBP
        //st7789v_write_data(0x04);
        st7789v_write_data(0x02); 
        //st7789v_write_data(0x08);
        //st7789v_write_data(0x45); 
        //HBP   
        //st7789v_write_data(0xa); 
        st7789v_write_data(0x14);
       
	
        //st7789v_write_data(0x0c);


	st7789v_write_cmd(0xB0); 
	st7789v_write_data(0x11); //set RGB interface and DE mode.
	//st7789v_write_data(0xF4); 
	st7789v_write_data(0xC0); 
	//st7789v_write_data(0x00); 
        st7789v_write_cmd(0x3a);
        st7789v_write_data(0x66); //18 RGB ,55-16BIT RGB


	//************************
	st7789v_write_cmd(0x11); 
	mdelay(120);      //Delay 120ms 

	st7789v_write_cmd(0x29); //display on
	st7789v_write_cmd(0x2c); 

}

static void st7789v_reset(void)
{
	gpio_set_value(RESET,1);
	mdelay(1);
	gpio_set_value(RESET,0);
	mdelay(100);
	gpio_set_value(RESET,1);
	mdelay(20);
}

static int cfg_gpio()
{
	int err;


	err = gpio_request(SPI_CS, "EIMD16"); //cs
	if (err) {
		printk(KERN_ERR "failed to request GPI0 for "
				"st7789v cs control\n");
		return err;
	}

	err = gpio_request(SPI_SDI, "EIMD18"); //data
	if (err) {
		printk(KERN_ERR "failed to request GPI1 for "
				"st7789v outdata control\n");
		return err;
	}

	err = gpio_request(SPI_CLK, "EIMD17"); //clk
	if (err) {
		printk(KERN_ERR "failed to request GPI8 for "
				"st7789v clk control\n");
		return err;
	}

	err = gpio_request(RESET, "EIMD19"); //reset
	if (err) {
		printk(KERN_ERR "failed to request GPI9 for "
				"st7789v reset control\n");
		return err;
	}

	gpio_direction_output(SPI_CS,1);
	gpio_direction_output(SPI_SDI,1);
	gpio_direction_output(SPI_CLK,1);
	gpio_direction_output(RESET,1);
	//s3c_gpio_cfgpin(SPI_CS, S3C_GPIO_OUTPUT);	//cs
	//s3c_gpio_setpull(SPI_CS, S3C_GPIO_PULL_UP);

	//s3c_gpio_cfgpin(SPI_SDI, S3C_GPIO_OUTPUT);	//outdata
	//s3c_gpio_setpull(SPI_SDI, S3C_GPIO_PULL_UP);

	//s3c_gpio_cfgpin(SPI_CLK,S3C_GPIO_OUTPUT);	//clk
	//s3c_gpio_setpull(SPI_CLK, S3C_GPIO_PULL_UP);

	//s3c_gpio_cfgpin(RESET, S3C_GPIO_OUTPUT);	//reset
	//s3c_gpio_setpull(RESET, S3C_GPIO_PULL_UP);

	return 0;
}

static void st7789v_release(void)
{
	set_cs(1);
	set_clk(1);
	set_outdata(1);
	gpio_free(SPI_CS);
	gpio_free(SPI_SDI);
	gpio_free(SPI_CLK);
	gpio_free(RESET);
}

static int __init st7789v_init(void)
{
	int err;	
	err = cfg_gpio();
	if(err < 0)
		return;
	st7789v_reset();
	set_st7789v();
	printk(KERN_INFO "Gzsd LCD st7789v init successfully!\n");
	return 0;

}

static void __exit st7789v_exit(void)
{
	st7789v_release();
}

module_init(st7789v_init);
module_exit(st7789v_exit);

MODULE_AUTHOR("Hcly Dao <hclydao@gmail.com>");
MODULE_DESCRIPTION("Gzsd210 st7789v driver");
MODULE_LICENSE("GPL");
