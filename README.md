++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

How to Clone:
	git clone https://github.com/carrrl1/NAOCAM.git
	cd NAOCAM
	git submodule init
	git submodule update
*Open bash.
*Cd to this directory.
*Run make to compile the documentation.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
HOW TO USE:
	*Download Naoqi C++ SDK and install it.
	*Install qibuild.
	*Install make if you haven’t.
	*On bash configure the project:
	 qibuild configure -c mytoolchain naocam
	*Compile the project:
 	 qibuild make -c mytoolchain
	*Copy img.jpg or whatever image of object you want to use inside ./sdk/bin
	*Cd to ./sdk/bin
	*Run the executable with:
	 ./naocam --pip <ip of Nao>

NOTE:
*If you have problems with DYLD libraries when loading the executable the export bin files from SDK:
	export DYLD_LIBRARY_PATH=/Users/path/to/SDK/lib/
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
