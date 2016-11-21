	MUSDK build tool
	================
NOTE: this is a non-formal version of Makefile. Please use the
	formal version found at the top (follow the README there).
	i.e. this build is only for internal use!

In order to build MUSDK, run the next steps:
1. create a file named '.env_params' under this folder
2. make sure the file contains the following:
	TOPDIR          = <path-to-the MUSDK-root-folder>
	KDIR            = <path-to-kernel-sources>
	TOPLIBGCC       = <path-to-GCC-lib>
	ARCH            = <arhcitecture>
	CROSS_COMPILE   = <path-to-cross-copiler>
3. run 'make targets' in order to get all available targets.
4. choose target and build
