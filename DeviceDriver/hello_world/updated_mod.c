#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>


MODULE_LICENSE("GPL");

int result;
struct file_operations mod_fops;


__init int module_start(void)
{
	printk("Hello World!!\n");
	result = register_chrdev(0, "HELLO", &mod_fops);
	printk("major number = %d\n", result);
	return 0;
}

__init void module_end(void) 
{
	printk("Goodbye World!!\n");
	unregister_chrdev(result, "HELLO");
}


module_init(module_start);
module_exit(module_end);

