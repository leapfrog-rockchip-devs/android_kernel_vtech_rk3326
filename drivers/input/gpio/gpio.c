
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/input/matrix_keypad.h>

int front_back_switch1_gpio;
int micro_distance_gpio;

int front_back_switch1_flag=0;
int micro_distance_level;
int front_back_switch1_old_level;
int micro_distance_old_level;
int front_back_switch1_level;
int micro_distance_mode;
int front_back_switch1_mode;
extern int s5k5e9power_on;
extern int Check_micro_distance;
extern int Adjusts5k5e9direction(int rotate);
struct control_gpio_info {
    int control_gpio;
    int gpio_enable_value;
    int gpio_direction;
    struct delayed_work gpio_delay_work;
    int input_register_flag;
};
 struct input_dev *keyinput = NULL;

#define READ_VDELAY 100    //100mS
#define key1  KEY_F19
#define key2  KEY_F20
static struct class *control_gpio = NULL; //¡À¨ª¨º????t?¨²¦Ì?
static ssize_t micro_distance_show(struct class *cls,struct class_attribute *attr, char *_buf)
{
	
//	micro_distance_level = gpio_get_value(micro_distance_gpio);
	if(s5k5e9power_on==0)
	{
		return sprintf(_buf, "0\n");
	}
	else{
		if(micro_distance_level==1)
			return sprintf(_buf, "1\n");
		else	
			return sprintf(_buf, "0\n");	
	}
}

static ssize_t micro_distance_store(struct class *cls,   struct class_attribute *attr, const char  *buf,size_t count)
{
	return count;
}
static CLASS_ATTR_RW(micro_distance);


static ssize_t front_back_switch1_show(struct class *cls,struct class_attribute *attr, char *_buf)
{
	int front_back_level;
	front_back_level = gpio_get_value(front_back_switch1_gpio);
	if(front_back_level==1)
		return sprintf(_buf, "1\n");
	else	
    	return sprintf(_buf, "0\n");	
		
}

static ssize_t front_back_switch1_store(struct class *cls,   struct class_attribute *attr, const char  *buf,size_t count)
{
	return count;
}
static CLASS_ATTR_RW(front_back_switch1);


void microdistanceprocess(void)
{
	int level;
	if(s5k5e9power_on==0)
	{
		return;
	}
		if(Check_micro_distance==1)
		{
				micro_distance_mode=4;
				Check_micro_distance=0;
			}
  	level = gpio_get_value(front_back_switch1_gpio);//gpio_get_value(micro_distance_gpio);
		switch(micro_distance_mode)
		{
		case 1:
					if(level!=micro_distance_old_level)
					{
			   		micro_distance_mode=2;
			   	}
		break;
		case 2:
			if(level!=micro_distance_old_level)
				{
					printk("frank debug micro_distance\n\n");
					micro_distance_level=level;
					micro_distance_old_level=level;
					input_report_key(keyinput, key2, 1);
				  input_sync(keyinput);
			   	micro_distance_mode=3;
				}
				else
				{
					micro_distance_mode=1;
				}
		break;
			case 3:
						input_report_key(keyinput, key2, 0);
				   	input_sync(keyinput);
						micro_distance_mode=1;
			break;
			case 4:
					printk("frank debug micro_distance1\n\n");
					micro_distance_level=level;
					micro_distance_old_level=level;
					input_report_key(keyinput, key2, 1);
				  input_sync(keyinput);
			   	micro_distance_mode=3;
			   	Check_micro_distance=0;
		break;
		default:
		micro_distance_mode=1;
		break;
	}
}

void front_back_switch1process(void)
{
	int level;
	if(front_back_switch1_flag!=0)
		front_back_switch1_flag--;
	else
	{
	level =gpio_get_value(micro_distance_gpio);// gpio_get_value(front_back_switch1_gpio);
	switch(front_back_switch1_mode)
	{
		case 1:
			   if(level!=front_back_switch1_old_level)
			   		front_back_switch1_mode=2;
		break;
		case 2:
				if(level!=front_back_switch1_old_level)
				{
					 printk("frank debug scan gpio\n\n");
					input_report_key(keyinput, key1, 1);
			         input_sync(keyinput);
					front_back_switch1_level=level;
					front_back_switch1_old_level=level;
					Adjusts5k5e9direction(front_back_switch1_level);
					front_back_switch1_mode=3;
				}
				else
				{
					front_back_switch1_mode=1;
				}
		break;
		case 3:
				if(level!=front_back_switch1_old_level)
				{
					printk("frank debug scan gpio3\n\n");
					input_report_key(keyinput, key1, 1);
			    input_sync(keyinput);
					front_back_switch1_level=level;
					front_back_switch1_old_level=level;
					Adjusts5k5e9direction(front_back_switch1_level);
					front_back_switch1_mode=4;
				}
				else
				{
					front_back_switch1_mode=1;
				}
				break;
		case 4:
						input_report_key(keyinput, key1, 0);
						input_sync(keyinput);
					  front_back_switch1_mode=5;
					  printk("frank debug scan gpio4\n\n");
			break;
		case 5:
					front_back_switch1_mode=1;
		break;
		
		default:
			front_back_switch1_mode=1;
		break;
	}
	}
//	 printk("frank debug scan gpio level=%d,front_back_switch1_mode=%d,front_back_switch1_old_level=%d,front_back_switch1_level=%d\n\n",level,front_back_switch1_mode,front_back_switch1_old_level,front_back_switch1_level);

}

                    
static void read_work(struct work_struct *work)
{
    
    struct control_gpio_info *gpio_info = container_of(work,struct control_gpio_info, gpio_delay_work.work);
     int ret=0;
 // printk("frank debug scan gpio\n\n");
  if(gpio_info->input_register_flag==1)
   {
   		gpio_info->input_register_flag=0;
   		 ret=input_register_device(keyinput);
   	
   }
    microdistanceprocess();
    front_back_switch1process();
	schedule_delayed_work(&gpio_info->gpio_delay_work,READ_VDELAY );
	
}

  struct input_dev *input = NULL;
static int gpio_control_probe(struct platform_device *pdev)
{
	  int ret=0;
    int gpio;
    enum of_gpio_flags flag;
    struct  control_gpio_info *gpio_info;
    struct device_node *control_gpio_node = pdev->dev.of_node;
    struct device *dev = &pdev->dev;
  
    //struct gpio_device *bq;
   
   //printk("gpio_control_probe \n\n\n\n\n");
	 // gpio_info = devm_kzalloc(&pdev->dev,sizeof(struct control_gpio_info *), GFP_KERNEL);
	  gpio_info = devm_kzalloc(&pdev->dev,sizeof(struct control_gpio_info), GFP_KERNEL);
	  if (!gpio_info) {
        dev_err(&pdev->dev, "devm_kzalloc failed!\n");
		  return -ENOMEM;
	   }
	   
	  	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}
	keyinput=input;
	input->name = "gpio-keypad1";	
	input->phys = "gpio-keypads1/input0";
	input->dev.parent = &pdev->dev;
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0002;
	input->id.product = 0x0002;
	input->id.version = 0x0200;
		__set_bit(EV_KEY, input->evbit);
		__set_bit(key1, input->keybit);
		__set_bit(key2, input->keybit);
  if (device_property_read_bool(dev, "autorepeat"))
		__set_bit(EV_REP, input->evbit);
	//ret = input_register_device(input);
	//if (ret) {
	//	dev_err(dev, "Unable to register input device: %d\n", ret);			
	//	return ret;
	//}
	   // set micro_distance_gpio
    gpio = of_get_named_gpio_flags(control_gpio_node, "micro_distance_gpio", 0, &flag);
    if (!gpio_is_valid(gpio)) {
        dev_err(&pdev->dev, "micro_distance_gpio: %d is invalid\n", gpio);
        return -ENODEV;
    }
    if (gpio_request(gpio, "micro_distance_gpio")) {
        dev_err(&pdev->dev, "micro_distance_gpio: %d request failed!\n", gpio);
        gpio_free(gpio);
        return -ENODEV;
    }
    gpio_info->control_gpio = gpio;
    micro_distance_gpio=gpio;
    //set   micro_distance_gpio as input 
    ret = gpio_direction_input(gpio_info->control_gpio);
		if (ret < 0) {
			dev_err(&pdev->dev,"micro_distance_gpio set direction fail\n");
			return -ENODEV;
		}
		
		//set front_back_switch1
	   gpio = of_get_named_gpio_flags(control_gpio_node, "front_back_switch1", 0, &flag);
	     if (!gpio_is_valid(gpio)) {
        dev_err(&pdev->dev, "front_back_switch1: %d is invalid\n", gpio);
        return -ENODEV;
    }
    if (gpio_request(gpio, "front_back_switch1")) {
        dev_err(&pdev->dev, "front_back_switch1: %d request failed!\n", gpio);
        gpio_free(gpio);
        return -ENODEV;
    }
     gpio_info->control_gpio = gpio;
     front_back_switch1_gpio = gpio;
	   gpio_direction_output(gpio_info->control_gpio,1);
	  
	    gpio = of_get_named_gpio_flags(control_gpio_node, "front_back_switch2", 0, &flag);
	     if (!gpio_is_valid(gpio)) {
        dev_err(&pdev->dev, "front_back_switch2 %d is invalid\n", gpio);
        return -ENODEV;
    }
    
    if (gpio_request(gpio, "front_back_switch2")) {
        dev_err(&pdev->dev, "front_back_switch2: %d request failed!\n", gpio);
        gpio_free(gpio);
        return -ENODEV;
    }
    gpio_info->control_gpio = gpio;
	  gpio_direction_output(gpio_info->control_gpio,0);
	    ret = gpio_direction_input(front_back_switch1_gpio);
		if (ret < 0) {
			dev_err(&pdev->dev,"micro_distance_gpio set direction fail\n");
			return -ENODEV;
		}  
		
	  control_gpio = class_create(THIS_MODULE, "control_gpio");
   	if (IS_ERR(control_gpio))
   	{
      	printk("control gpio fail.\n");
       	ret = -ENOMEM;
   	}
   	 // creat icro distance class 
  	ret = class_create_file(control_gpio,&class_attr_micro_distance);	
		if (ret) 
		{
			printk(KERN_ERR "%s:Fail to creat micro distance class file\n", __func__);
			return ret;
	  }
	  
	  // creat front back switch class
			ret = class_create_file(control_gpio,&class_attr_front_back_switch1);	
		if (ret) 
		{
			printk(KERN_ERR "%s:Fail to creat front back switch class file\n", __func__);
			return ret;
	  }
	gpio_info->input_register_flag=1;
	front_back_switch1_flag=0;
    INIT_DELAYED_WORK(&gpio_info->gpio_delay_work, read_work);
    schedule_delayed_work(&gpio_info->gpio_delay_work,READ_VDELAY+1000 );
    printk("===========control_gpio_probe end===========\n");
	  return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id gpio_control_of_match[] = {
	{ .compatible = "gpio-control", },
	{ }
};
MODULE_DEVICE_TABLE(of, gpio_control_of_match);
#endif

static struct platform_driver __refdata gpio_control_driver = {
	.driver = {
		.name = "gpio-control",
		.of_match_table = of_match_ptr(gpio_control_of_match),
	},
	.probe = gpio_control_probe,
};

module_platform_driver(gpio_control_driver);

MODULE_AUTHOR("frank cai <frank-cai@szxizhuo.com>");
MODULE_DESCRIPTION("GPIO control");
MODULE_LICENSE("GPL v2");




