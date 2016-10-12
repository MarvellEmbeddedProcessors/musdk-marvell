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
+
+				eth0@010000 {
+					interrupts = <0x0 0x25 0x4 0x0 0x29 0x4 0x0 0x2d 0x4 0x0 0x31 0x4 0x0 0x35 0x4>;
+					port-id = <0x0>;
+					emac-data = <0x24>;
+					status = "okay";
+				};
+
+				eth1@020000 {
+					interrupts = <0x0 0x26 0x4 0x0 0x2a 0x4 0x0 0x2e 0x4 0x0 0x32 0x4 0x0 0x36 0x4>;
+					port-id = <0x1>;
+					emac-data = <0x25>;
+					status = "okay";
+				};
+			};

			ppv22@000000 {
				compatible = "marvell,mv-pp22";
				reg = <0x0 0x90000 0x120000 0x9000 0x129000 0x600 0x12a000 0x200 0x12a200 0x200 0x12a400 0x200 0x12a600 0x200 0x12b000 0x1000 0x130000 0x6000 0x130400 0x200 0x130e00 0x100 0x130f00 0x100 0x441000 0x1000>;
