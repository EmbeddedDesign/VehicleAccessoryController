Build instrutions Ubuntu:

make
sudo cp cp210x_vehicle_accessory_controller.ko /lib/modules/$(uname -r)/kernel/drivers/usb/serial
sudo insmod /lib/modules/$(uname -r)/kernel/drivers/usb/serial/usbserial.ko
sudo insmod cp210x_vehicle_accessory_controller.ko
echo 'cp210x_vehicle_accessory_controller.ko' | sudo tee -a /etc/modules
sudo depmod
reboot