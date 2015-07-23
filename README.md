# FIRMWARE FOR RUSK R002 -- PRODUCTION RELEASE

This application uses the our libraries from [GitHub](https://github.com/foxfire-online/):

* [foxfire-online/RUSK-r002](https://github.com/foxfire-online/RUSK-r002)
* [foxfire-online/SPARK_EmonLib](https://github.com/foxfire-online/SPARK_EmonLib)
* [foxfire-online/SPARK_BMP085](https://github.com/foxfire-online/SPARK_BMP085)
* [foxfire-online/SPARK_Si1132](https://github.com/foxfire-online/SPARK_Si1132)
* [foxfire-online/SPARK_Si70xx](https://github.com/foxfire-online/SPARK_Si70xx)
* [foxfire-online/SPARK_MPU9150](https://github.com/foxfire-online/SPARK_MPU9150)

## QUICK START GUIDE

You import the libraries above through [Build](https://build.spark.io/login), so login or sign up first.

Import "foxfire-online/RUSK-r002" first. Once complete, search for the RUSK-Basic.ino file, once selected you will be given the option to "use this example".  Apply the example, and it will create a new application for you called RUSK-BASIC.

After you have imported the example, import the rest of the libraries, and add them to the RUSK-BASIC application.

You will need all of them.

Once complete, "save" your project and "verify" the code. Upon successful verification, you have two ways of getting the firmware into your Core/Photon.

### EASY WAY -- Take note sometimes this takes a couple tries

You can flash it "over the air". It just works!

### RELIABLE WAY -- You will need to set up the the Core/Photon command line utilities first (Node.js, dfu-util, etc)

Once your environment has been set up, click the "cloud" icon in the Build "code" interface to download your compiled firmware.  The file is typically called firmware.bin (if you feel this to be too complex, you can also download a pre-compiled copy from our website).

If you have not already done so, please register and configure your Core/Photon:

```
particle setup
```

Next, switch to DFU mode (LED flashes yellow), and you simply "flash" your Core/Photon with the firmware:

```
particle flash --usb "X:path\to\firmware.bin"
```

# HISTORY
2015/JUL/23  - First version (FOXFIRE)
