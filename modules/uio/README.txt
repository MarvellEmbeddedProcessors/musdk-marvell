		Building MUSDK-UIO Kernel Module
		================================
- Make sure KDIR is defined correctly in the environment.
- Make sure the Linux Kernel is built with 'CONFIG_UIO' flag enabled.
- Before building and running the module, make sure the below change in DTS
	file is applied. please note that for every IO that is needed in User-Space
	operation, an appropriate entry should be created in the DTS under the uio
	entry.
- Build the musdk_uio module by running the following:
> make
> make DST=<> install
- insert the MUSDK-UIO module by 'insmod musdk_uio.ko'

DTS file change:
----------------
					phandle = <0x26>;
				};
			};

+			musdk_uio {
+				compatible = "marvell,musdk-uio";
+				status = "okay";
+			};

			ppv22@000000 {
				compatible = "marvell,mv-pp22";
				reg = <0x0 0x90000 0x120000 0x9000 0x129000 0x600 0x12a000 0x200 0x12a200 0x200 0x12a400 0x200 0x12a600 0x200 0x12b000 0x1000 0x130000 0x6000 0x130400 0x200 0x130e00 0x100 0x130f00 0x100 0x441000 0x1000>;
