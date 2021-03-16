/*
 * Copyright (C) 2019 Pavel Fiala / gpio sysfs driver
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place, Suite 330, Boston, MA 02111-1307 USA
 */

 // System Linux includes ...
 // -------------------------
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/bitops.h>
#include <linux/iopoll.h>
#include <linux/semaphore.h>

#define CONST_NAME_BUF_SIZE 32	       // Default value - 32 ...

// Global definitions ...
// ----------------------
static int g_dev_index;
static struct platform_driver gpio_platform_driver;
static struct list_head g_dev_list;
static struct semaphore g_dev_probe_sem;

// struct altera_gpio_dev ...
// --------------------------
struct altera_gpio_dev{
  int id;     			     /* altera_gpio_dev id in list */
  int g_gpio_reg_hw_addr; 	     /* desc register base address*/
  int g_gpio_reg_hw_size; 	     /* desc register region size*/
  void __iomem *g_ioremap_gpio_addr; /* register remap addres */
  struct device *pdev_dev; 	     /* altera_gpio_dev device structure */
  struct semaphore dev_sem;          /* altera_gpio_dev semaphore structure */
  struct list_head dev_list;         /* altera_gpio_dev list_head structure */
};

static ssize_t altera_gpio_show(struct device *dev, struct device_attribute *attr, char *buf){
  
    u8 value = 0x00;
  
    struct altera_gpio_dev *g_altera_gpio_dev = (struct altera_gpio_dev *)dev_get_drvdata(dev);
    
    /* acquire the probe lock */
    if(down_interruptible(&g_altera_gpio_dev->dev_sem)){
       pr_info("altera_gpio_store sem interrupted - exit \n");
       return -ERESTARTSYS;
    }
    
    value = ioread8(g_altera_gpio_dev->g_ioremap_gpio_addr);
    
    up(&g_altera_gpio_dev->dev_sem);
    
    return scnprintf(buf, 4,"%d", value);
}

static ssize_t altera_gpio_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
  
    u8 value = 0x00;
  
    struct altera_gpio_dev *g_altera_gpio_dev = (struct altera_gpio_dev *)dev_get_drvdata(dev);
    
    /* acquire the probe lock */
    if(down_interruptible(&g_altera_gpio_dev->dev_sem)){
       pr_info("altera_gpio_store sem interrupted - exit \n");
       return -ERESTARTSYS;
    }
    
    if(buf == NULL) {
       pr_err("error - string must not be NULL \n");
       return -EINVAL;
    }
    
    if(kstrtou8(buf, 10, &value) < 0) {
       pr_err("error - could not convert string to integer\n");
       return -EINVAL;
    }
        
    iowrite8(value, g_altera_gpio_dev->g_ioremap_gpio_addr);
    
    up(&g_altera_gpio_dev->dev_sem);

    return count;
}

// Write permission for OTHERS ...
// -------------------------------
// https://stackoverflow.com/questions/40776170/cannot-set-attribute-permissions-to-0666-in-sysfs
#undef VERIFY_OCTAL_PERMISSIONS
#define VERIFY_OCTAL_PERMISSIONS(perms) (perms)
DEVICE_ATTR(altera_gpio, S_IWUGO | S_IRUGO /*0644*/, altera_gpio_show, altera_gpio_store);

// Platform_probe function ...
// ---------------------------
static int platform_probe(struct platform_device *pdev){
  
  int ret_val = 0;
  
  struct resource *r = 0;
  struct resource *altera_gpio_mem_region = 0;
  struct altera_gpio_dev *g_altera_gpio_dev = 0;
  
  char gpio_name_region[CONST_NAME_BUF_SIZE];

  ret_val = -EBUSY;   /* Device or resource busy - 16 */

  /* acquire the probe lock */
  if(down_interruptible(&g_dev_probe_sem)){
    return -ERESTARTSYS;
  }

  /* allocate mem for altera_gpio_dev struct */
  g_altera_gpio_dev = kzalloc(sizeof(struct altera_gpio_dev),GFP_KERNEL);
  if(g_altera_gpio_dev == NULL){
    pr_err("error - kzalloc failed - could not allocate memory \n");
    ret_val = -ENOMEM;    /* Out of memory - 12 */
    goto bad_exit_kfree_0;
  }
  
  scnprintf(gpio_name_region,CONST_NAME_BUF_SIZE,"altera_gpio%d",
             g_dev_index);
   
  g_altera_gpio_dev->id = g_dev_index;
   
  ret_val = -EINVAL;  /* Invalid argument - 22 */
   
  // -------------------------------------------------
  // Get resource 0 - GPIO registers memory region ...
  // -------------------------------------------------
  r = platform_get_resource(pdev,IORESOURCE_MEM,0);
  
  if(r == NULL){
    pr_err("error - IORESOURCE_MEM 0 does not exist \n");
    goto bad_exit_return;
  }

  g_altera_gpio_dev->g_gpio_reg_hw_addr = r->start;         /* gpio register base address*/
  g_altera_gpio_dev->g_gpio_reg_hw_size = resource_size(r); /* gpio register region size*/

  ret_val = -EBUSY;  /* Device or resource busy - 16 */
  
  // Reserve GPIO memory region ...
  // ------------------------------
  altera_gpio_mem_region = request_mem_region(g_altera_gpio_dev->g_gpio_reg_hw_addr,
                                              g_altera_gpio_dev->g_gpio_reg_hw_size,
                                              gpio_name_region); // "altera_gpio%d"
  if(altera_gpio_mem_region == NULL){
     pr_err("error - request_mem_region failed - altera_gpio_mem_region \n");
     goto bad_exit_return;
  }
  
  // Ioremap GPIO memory region ...
  // ------------------------------
  g_altera_gpio_dev->g_ioremap_gpio_addr = ioremap(g_altera_gpio_dev->g_gpio_reg_hw_addr,
                                                   g_altera_gpio_dev->g_gpio_reg_hw_size);

  if(g_altera_gpio_dev->g_ioremap_gpio_addr == NULL){
     pr_err("error - ioremap failed - altera_gpio_mem_region \n");
     goto bad_exit_release_mem_region;
  }
  
  g_altera_gpio_dev->pdev_dev = &pdev->dev; // Set pdev_dev ...
  sema_init(&g_altera_gpio_dev->dev_sem,1);

  ret_val = -EBUSY; /* Device or resource busy - 16 */
  
  // Create the sysfs entries ...
  // ----------------------------
  ret_val = device_create_file(g_altera_gpio_dev->pdev_dev, &dev_attr_altera_gpio);
  if (ret_val != 0) {
      pr_err("error - failed to create irq_delays sysfs entry");
      goto bad_exit_iounmap_region;
  }
  
  // ------------------------------
  // Add device to list finally ...
  // ------------------------------
  INIT_LIST_HEAD(&g_altera_gpio_dev->dev_list);
  list_add(&g_altera_gpio_dev->dev_list,&g_dev_list);
  
  g_dev_index++;

  // ---------------------
  // Set platform data ...
  // ---------------------
  platform_set_drvdata(pdev,g_altera_gpio_dev);
  
  up(&g_dev_probe_sem);
  // Return success ...
  // ------------------
  return 0;
  
  // ----------------------------
  // Error handling goes here ...
  // ----------------------------
  bad_exit_iounmap_region:
    iounmap(g_altera_gpio_dev->g_ioremap_gpio_addr);
  bad_exit_release_mem_region:
    release_mem_region(g_altera_gpio_dev->g_gpio_reg_hw_addr,
                       g_altera_gpio_dev->g_gpio_reg_hw_size);
  bad_exit_return:
    kfree(g_altera_gpio_dev);
  bad_exit_kfree_0:
    up(&g_dev_probe_sem);
    return ret_val;
}

// Platform_remove function ...
// ---------------------------
static int platform_remove(struct platform_device *pdev){

  struct altera_gpio_dev *g_altera_gpio_dev = 0;

  /* acquire the probe lock */
  if(down_interruptible(&g_dev_probe_sem)){
    return -ERESTARTSYS;
  }

  g_altera_gpio_dev = (struct altera_gpio_dev *)platform_get_drvdata(pdev);
  list_del_init(&g_altera_gpio_dev->dev_list);
  device_remove_file(g_altera_gpio_dev->pdev_dev, &dev_attr_altera_gpio);
  
  iounmap(g_altera_gpio_dev->g_ioremap_gpio_addr);
  release_mem_region(g_altera_gpio_dev->g_gpio_reg_hw_addr,
                     g_altera_gpio_dev->g_gpio_reg_hw_size);
  g_dev_index--;

  kfree(g_altera_gpio_dev);

  up(&g_dev_probe_sem);
  return 0;
}

// Device Tree compatibility altr,altera-gpio-st-1.0 ...
// -----------------------------------------------------
static struct of_device_id driver_dt_ids[] ={
  {
    .compatible = "altr,altera-gpio-1.0"},
  { /* END of the DT table */}
};

MODULE_DEVICE_TABLE(of,driver_dt_ids);

static struct platform_driver gpio_platform_driver = {
  .probe = platform_probe,    // Platform probe ...
  .remove = platform_remove,  // Platform remove ...
  .driver = {
            .name = "altera_gpio",
            .owner = THIS_MODULE,
            .of_match_table = driver_dt_ids,
          },
};

static int altera_gpio_init(void){

  int ret_val = 0;

  // Init list ...
  INIT_LIST_HEAD(&g_dev_list);

  /* Init semaphore as MUTEX ...
     Using semaphore as a MUTEX, the value of semaphore is initialized to 1.
     So at any give time only one process can execute the critical section ...
  */
  sema_init(&g_dev_probe_sem,1);

  g_dev_index = 0;

  // Register platform driver ...
  // ----------------------------
  ret_val = platform_driver_register(&gpio_platform_driver);
  if(ret_val != 0){
    pr_err("error - platform_driver_register returned %d \n",ret_val);
    return ret_val;
  }

  return 0;
}

static void altera_gpio_exit(void){

  // Unregister platform driver ...
  // ------------------------------
  platform_driver_unregister(&gpio_platform_driver);

}

module_init(altera_gpio_init);
module_exit(altera_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pavel Fiala <pavelfpl@gmail.com");
MODULE_DESCRIPTION("Intel / Altera general GPIO driver");
MODULE_VERSION("1.3");