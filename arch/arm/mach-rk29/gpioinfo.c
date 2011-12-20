#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/sysdev.h>
#include <linux/proc_fs.h>

#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/rk29_iomap.h>
#include <mach/iomux.h>
#include <asm/gpio.h>


#define to_rk29_gpio_chip(c) container_of(c, struct rk29_gpio_chip, chip)

struct rk29_gpio_chip {
	struct gpio_chip        chip;
	unsigned short id;
	short irq;
	unsigned char  __iomem	*regbase;	/* Base of register bank */
	struct clk *clk;
	u32 suspend_wakeup;
	u32 saved_wakeup;
};

static void rk29_gpiolib_set(struct gpio_chip *chip, unsigned offset, int val);
static int rk29_gpiolib_get(struct gpio_chip *chip, unsigned offset);
static int rk29_gpiolib_direction_output(struct gpio_chip *chip,unsigned offset, int val);
static int rk29_gpiolib_direction_input(struct gpio_chip *chip,unsigned offset);
static int rk29_gpiolib_PullUpDown(struct gpio_chip *chip, unsigned offset, unsigned enable);
static int rk29_gpiolib_to_irq(struct gpio_chip *chip,unsigned offset);

#define RK29_GPIO_CHIP(ID)			\
	{								\
		.chip = {						\
			.label            = "gpio" #ID,			\
			.direction_input  = rk29_gpiolib_direction_input, \
			.direction_output = rk29_gpiolib_direction_output, \
			.get              = rk29_gpiolib_get,		\
			.set              = rk29_gpiolib_set,		\
			.pull_updown      = rk29_gpiolib_PullUpDown,	\
			.to_irq           = rk29_gpiolib_to_irq,	\
			.base             = PIN_BASE + ID*NUM_GROUP,	\
			.ngpio            = NUM_GROUP,			\
		},							\
		.id = ID, \
		.irq = IRQ_GPIO##ID, \
		.regbase = (unsigned char __iomem *) RK29_GPIO##ID##_BASE, \
	}

static struct rk29_gpio_chip rk29gpio_chip[] = {
	RK29_GPIO_CHIP(0),
	RK29_GPIO_CHIP(1),
	RK29_GPIO_CHIP(2),
	RK29_GPIO_CHIP(3),
	RK29_GPIO_CHIP(4),
	RK29_GPIO_CHIP(5),
	RK29_GPIO_CHIP(6),
};

static inline void rk29_gpio_write(unsigned char  __iomem	*regbase, unsigned int regOff,unsigned int val)
{
	__raw_writel(val,regbase + regOff);
}

static inline unsigned int rk29_gpio_read(unsigned char  __iomem	*regbase, unsigned int regOff)
{
	return __raw_readl(regbase + regOff);
}

static inline void rk29_gpio_bitOp(unsigned char  __iomem	*regbase, unsigned int regOff,unsigned int mask,unsigned char opFlag)
{
	unsigned int valTemp = 0;
	
	if(opFlag == 0)//对寄存器相应位进行与0操作
	{
		valTemp = rk29_gpio_read(regbase,regOff);  
		valTemp &= (~mask);;
		rk29_gpio_write(regbase,regOff,valTemp);
	}
	else if(opFlag == 1)//对寄存器相应位进行或1操作
	{
		valTemp = rk29_gpio_read(regbase,regOff);
		valTemp |= mask;
		rk29_gpio_write(regbase,regOff,valTemp);
	}
}

static inline  struct gpio_chip *pin_to_gpioChip(unsigned pin)
{
	if(pin < PIN_BASE)
		return NULL;
	
	pin -= PIN_BASE;
	pin /= NUM_GROUP;
	if (likely(pin < MAX_BANK))
		return &(rk29gpio_chip[pin].chip);
	return NULL;
}

static inline unsigned  pin_to_mask(unsigned pin)
{
	if(pin < PIN_BASE)
		return 0;
	pin -= PIN_BASE;
	return 1ul << (pin % NUM_GROUP);
}

static inline unsigned  offset_to_mask(unsigned offset)
{
	return 1ul << (offset % NUM_GROUP);
}

static int GPIOSetPinLevel(struct gpio_chip *chip, unsigned int mask,eGPIOPinLevel_t level)
{
	struct rk29_gpio_chip *rk29_gpio = to_rk29_gpio_chip(chip);
	unsigned char  __iomem	*gpioRegBase = rk29_gpio->regbase;
	unsigned long flags;

	if(!rk29_gpio || !gpioRegBase)
	{
		return -1;
	}

	local_irq_save(flags);
	rk29_gpio_bitOp(gpioRegBase,GPIO_SWPORT_DDR,mask,1);
	rk29_gpio_bitOp(gpioRegBase,GPIO_SWPORT_DR,mask,level);
	local_irq_restore(flags);

	return 0;
}

static int GPIOGetPinLevel(struct gpio_chip *chip, unsigned int mask)
{
	unsigned int valTemp;
	struct rk29_gpio_chip *rk29_gpio = to_rk29_gpio_chip(chip);
	unsigned char  __iomem	*gpioRegBase = rk29_gpio->regbase;

	if(!rk29_gpio || !gpioRegBase)
	{
		return -1;
	}
	
	valTemp = rk29_gpio_read(gpioRegBase,GPIO_EXT_PORT);
	return ((valTemp & mask) != 0);
}

static int GPIOGetPinDirection(struct rk29_gpio_chip *rk29_gpio, unsigned int offset)
{
	unsigned char  __iomem	*gpioRegBase = rk29_gpio->regbase;
	unsigned long flags;
	unsigned int temp;

	if(!rk29_gpio || !gpioRegBase)
	{
		return -1;
	}

	local_irq_save(flags);
	temp = rk29_gpio_read(gpioRegBase, GPIO_SWPORT_DDR);
	local_irq_restore(flags);

	return ((temp & offset_to_mask(offset)) != 0);
}

static int GPIOSetPinDirection(struct gpio_chip *chip, unsigned int mask,eGPIOPinDirection_t direction)
{
	struct rk29_gpio_chip *rk29_gpio = to_rk29_gpio_chip(chip);
	unsigned char  __iomem	*gpioRegBase = rk29_gpio->regbase;
	unsigned long flags;

	if(!rk29_gpio || !gpioRegBase)
	{
		return -1;
	}

	local_irq_save(flags);
	rk29_gpio_bitOp(gpioRegBase,GPIO_SWPORT_DDR,mask,direction);
	/* Enable debounce may halt cpu on wfi, disable it by default */
	//rk29_gpio_bitOp(gpioRegBase,GPIO_DEBOUNCE,mask,1);
	local_irq_restore(flags);

	return 0;
}

static int GPIOEnableIntr(struct gpio_chip *chip, unsigned int mask)
{
	struct rk29_gpio_chip *rk29_gpio = to_rk29_gpio_chip(chip);
	unsigned char  __iomem	*gpioRegBase = rk29_gpio->regbase;

	if(!rk29_gpio || !gpioRegBase)
	{
		return -1;
	}
	
	rk29_gpio_bitOp(gpioRegBase,GPIO_INTEN,mask,1);

	return 0;
}

static int GPIODisableIntr(struct gpio_chip *chip, unsigned int mask)
{
	struct rk29_gpio_chip *rk29_gpio = to_rk29_gpio_chip(chip);
	unsigned char  __iomem	*gpioRegBase = rk29_gpio->regbase;

	if(!rk29_gpio || !gpioRegBase)
	{
		return -1;
	}

	rk29_gpio_bitOp(gpioRegBase,GPIO_INTEN,mask,0);

	return 0;
}

static int GPIOSetIntrType(struct gpio_chip *chip, unsigned int mask, eGPIOIntType_t IntType)
{
	struct rk29_gpio_chip *rk29_gpio = to_rk29_gpio_chip(chip);
	unsigned char  __iomem	*gpioRegBase = rk29_gpio->regbase;

	if(!rk29_gpio || !gpioRegBase)
	{
		return -1;
	}
	
	switch ( IntType )
	{
	    case GPIOLevelLow:
			rk29_gpio_bitOp(gpioRegBase,GPIO_INT_POLARITY,mask,0);	
			rk29_gpio_bitOp(gpioRegBase,GPIO_INTTYPE_LEVEL,mask,0);	
			break;
	    case GPIOLevelHigh:
			rk29_gpio_bitOp(gpioRegBase,GPIO_INTTYPE_LEVEL,mask,0);	
			rk29_gpio_bitOp(gpioRegBase,GPIO_INT_POLARITY,mask,1);	
			break;
	    case GPIOEdgelFalling:
			rk29_gpio_bitOp(gpioRegBase,GPIO_INTTYPE_LEVEL,mask,1);	
			rk29_gpio_bitOp(gpioRegBase,GPIO_INT_POLARITY,mask,0);	
			break;
	    case GPIOEdgelRising:
			rk29_gpio_bitOp(gpioRegBase,GPIO_INTTYPE_LEVEL,mask,1);	
			rk29_gpio_bitOp(gpioRegBase,GPIO_INT_POLARITY,mask,1);	
			break;
		default:
			return(-1);
	}
	 return(0);
}

static int gpio_irq_set_wake(unsigned int irq, unsigned int on)
{
	unsigned int pin = irq_to_gpio(irq);
	unsigned bank = (pin - PIN_BASE) / NUM_GROUP;
	struct rk29_gpio_chip *rk29_gpio;
	unsigned mask = pin_to_mask(pin);

	if (unlikely(bank >= MAX_BANK))
		return -EINVAL;

	rk29_gpio = &rk29gpio_chip[bank];
	if (on)
		rk29_gpio->suspend_wakeup |= mask;
	else
		rk29_gpio->suspend_wakeup &= ~mask;

	set_irq_wake(rk29_gpio->irq, on);

	return 0;
}

static int gpio_irq_type(unsigned irq, unsigned type)
{
	unsigned int pin = irq_to_gpio(irq);
	struct gpio_chip *chip = pin_to_gpioChip(pin);
	unsigned	mask = pin_to_mask(pin);
	
	if(!chip || !mask)
		return -EINVAL;
	//设置为中断之前，必须先设置为输入状态
	GPIOSetPinDirection(chip,mask,GPIO_IN);
	
	switch (type) {
		case IRQ_TYPE_NONE:
			break;
		case IRQ_TYPE_EDGE_RISING:
			GPIOSetIntrType(chip,mask,GPIOEdgelRising);
			break;
		case IRQ_TYPE_EDGE_FALLING:
			GPIOSetIntrType(chip,mask,GPIOEdgelFalling);
			break;
		case IRQ_TYPE_EDGE_BOTH:
			break;
		case IRQ_TYPE_LEVEL_HIGH:
			GPIOSetIntrType(chip,mask,GPIOLevelHigh);
			break;
		case IRQ_TYPE_LEVEL_LOW:
			GPIOSetIntrType(chip,mask,GPIOLevelLow);
			break;
		default:
			return -EINVAL;
	}

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		__set_irq_handler_unlocked(irq, handle_level_irq);
	else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		__set_irq_handler_unlocked(irq, handle_edge_irq);

	return 0;
}

static int GPIOAckIntr(struct gpio_chip *chip, unsigned int mask)
{
	struct rk29_gpio_chip *rk29_gpio = to_rk29_gpio_chip(chip);
	unsigned char  __iomem	*gpioRegBase = rk29_gpio->regbase;

	if(!rk29_gpio || !gpioRegBase)
	{
		return -1;
	}
	
	rk29_gpio_bitOp(gpioRegBase,GPIO_PORTS_EOI,mask,1);
	return 0;
}

static void gpio_irq_unmask(unsigned irq)
{
	unsigned int pin = irq_to_gpio(irq);
	struct gpio_chip *chip = pin_to_gpioChip(pin);
	unsigned	mask = pin_to_mask(pin);

	if(chip && mask)
		GPIOEnableIntr(chip,mask);
}

static void gpio_irq_mask(unsigned irq)
{
	unsigned int pin = irq_to_gpio(irq);
	struct gpio_chip *chip = pin_to_gpioChip(pin);
	unsigned	mask = pin_to_mask(pin);

	if(chip && mask)
		GPIODisableIntr(chip,mask);
}

static void gpio_ack_irq(u32 irq)
{
	unsigned int pin = irq_to_gpio(irq);
	struct gpio_chip *chip = pin_to_gpioChip(pin);
	unsigned	mask = pin_to_mask(pin);

	if(chip && mask)
		GPIOAckIntr(chip,mask);
}

static int GPIOPullUpDown(struct gpio_chip *chip, unsigned int offset, unsigned enable)
{
	unsigned int temp = 0;
	struct rk29_gpio_chip *rk29_gpio = to_rk29_gpio_chip(chip);
	unsigned char  __iomem *pGrfRegBase = (unsigned char  __iomem *)RK29_GRF_BASE;
	unsigned long flags;

	if(!rk29_gpio || !pGrfRegBase)
	{
		return -1;
	}
	
	if(offset >= 32)
	{
		return -1;
	}

	local_irq_save(flags);
	temp = __raw_readl(pGrfRegBase + 0x78 +(rk29_gpio->id)*4);
	if(!enable)
		temp |= 1<<offset;
	else
		temp &= ~(1<<offset);

	__raw_writel(temp,pGrfRegBase + 0x78 +(rk29_gpio->id)*4);
	local_irq_restore(flags);

	return 0;
}

static int GPIOGetPull(struct gpio_chip *chip, unsigned int offset)
{
        unsigned int temp = 0;
        struct rk29_gpio_chip *rk29_gpio = to_rk29_gpio_chip(chip);
        unsigned char  __iomem *pGrfRegBase = (unsigned char  __iomem *)RK29_GRF_BASE;
        unsigned long flags;

        if(!rk29_gpio || !pGrfRegBase)
        {
                return -1;
        }

        if(offset >= 32)
        {
                return -1;
        }

        local_irq_save(flags);
        temp = __raw_readl(pGrfRegBase + 0x78 +(rk29_gpio->id)*4);
        local_irq_restore(flags);

        return ((temp & (1<<offset)) != 0);
}



static int rk29_gpiolib_direction_output(struct gpio_chip *chip,unsigned offset, int val)
{
	unsigned	mask = offset_to_mask(offset);
	
	if(GPIOSetPinDirection(chip,mask,GPIO_OUT) == 0)
	{
		return GPIOSetPinLevel(chip,mask,val);
	}
	else
	{
		return -1;
	}
}

static int rk29_gpiolib_direction_input(struct gpio_chip *chip,unsigned offset)
{
	unsigned	mask = offset_to_mask(offset);
	
	return GPIOSetPinDirection(chip,mask,GPIO_IN);
}


static int rk29_gpiolib_get(struct gpio_chip *chip, unsigned offset)
{
	unsigned	mask = offset_to_mask(offset);
	
	return GPIOGetPinLevel(chip,mask);
}

static void rk29_gpiolib_set(struct gpio_chip *chip, unsigned offset, int val)
{
	unsigned	mask = offset_to_mask(offset);
	
	GPIOSetPinLevel(chip,mask,val);
}

static int rk29_gpiolib_PullUpDown(struct gpio_chip *chip, unsigned offset, unsigned enable)
{
	return GPIOPullUpDown(chip, offset, enable);
}

static int rk29_gpiolib_to_irq(struct gpio_chip *chip,
						unsigned offset)
{
    struct rk29_gpio_chip *rk29_gpio = to_rk29_gpio_chip(chip);

    if(!rk29_gpio)
    {
    	 return -1;
    }

    return offset + NR_IRQS;
}

static int rk29_gpio_show(struct seq_file *s, void *v)
{
	int status = -EINVAL;
	int i;

	for (i = 0; i < ARRAY_SIZE(rk29gpio_chip); ++i)
	{
		int offset;
		int gpio;
		struct rk29_gpio_chip *chip = &rk29gpio_chip[i];
		for (offset = 0; offset < NUM_GROUP; ++offset)
		{
			gpio = i * NUM_GROUP + offset;
			status = gpio_request(gpio, "gpioinfo");
		
			if (status == 0)
			{
				gpio_free(gpio);
				seq_printf(s, "gpio %d: is free\n", gpio);
				seq_printf(s, "\tpull: %d\n", GPIOGetPull((struct gpio_chip*)chip, offset));
			}
			else
			{
				seq_printf(s, "gpio %d: is requested\n", gpio);
				seq_printf(s, "\tdirection: %d\n", GPIOGetPinDirection(chip, offset));
				seq_printf(s, "\tvalue: %d\n", rk29_gpiolib_get((struct gpio_chip*)chip, offset));
			}
		}
	}

	return 0;
}

static int gpioinfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rk29_gpio_show, NULL);
}

static struct file_operations gpioinfo_proc_ops = {
	.owner	= THIS_MODULE,
	.open	= gpioinfo_proc_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release= seq_release,
};

static struct proc_dir_entry *gpioinfo_proc_entry = 0;
#define GPIOINFO_ENTRY_NAME "gpioinfo"

static int __init gpioinfo_init(void)
{
	gpioinfo_proc_entry = create_proc_entry(GPIOINFO_ENTRY_NAME, 0, NULL);
	if (gpioinfo_proc_entry)
		gpioinfo_proc_entry->proc_fops = &gpioinfo_proc_ops;

	return 0;
}

static void __exit gpioinfo_exit(void)
{
	remove_proc_entry(GPIOINFO_ENTRY_NAME, NULL);
}

module_init(gpioinfo_init)
module_exit(gpioinfo_exit)

MODULE_AUTHOR("<dayong1111@gmail.com>");
MODULE_DESCRIPTION("gpioinfo extractor");
MODULE_LICENSE("GPL");

