
---

# How to Clone:

* `git clone https://github.com/carrrl1/NAOCAM.git`
* `cd NAOCAM`
* `git submodule init`
* `git submodule update`

---

# How to compile the documentation:

* Open bash.
* Cd to this directory.
* Run `make` to compile the documentation.

---

# How to compile:

* Download Naoqi C++ SDK and install it.
* Install qibuild. Follow the steps provided by Aldebaran in this [link][1].
* Install cmake if you haven’t.
* Cd to NaoCam.
* On bash configure the project:

	 `qibuild configure`

* Compile the project:

 	 `qibuild make`

* Copy img.jpg or whatever image of object you want to use inside ./sdk/bin
* Cd to ./sdk/bin
* Run the executable with:

	 `./naocam --pip <ip of Nao>`

---

## NOTE:
* If you have problems with DYLD libraries when loading the executable the export bin files from SDK:
	`<export DYLD_LIBRARY_PATH=/Users/path/to/SDK/lib/>

---

[1]: http://doc.aldebaran.com/2-1/dev/cpp/install_guide.html "Aldebaran Documentation C++ SDK Installation"
