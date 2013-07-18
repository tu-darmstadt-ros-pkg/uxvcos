		    HUDAQLIB

1, How to build and install

1.1, Build and install library
type 
  make

if build seems to be OK, type 
  make install


1.2, Improve PCI listing  
optionally you could patch /usr/share/pci.ids. 
type 
  lspci
  
If you don't see your device correctly, like this:
  "00:08.0 Signal processing controller: Unknown device 186c:0624"
  
this is correct:  
  "00:08.0 Signal processing controller: Humusoft, s.r.o. MF624 Multifunction I/O Card"

you could try to type:  
  make patch

1.3. Configure driver
It is possible to remove unwanted hardware support and compile only neccessary
driver(s).

 Edit "makefile" and specify which components you need. Comment out lines for
unnecessary drivers.

#optional components
MF61X   := 1
MF62X   := 1
PCI1753 := 1
PCD7004 := 1
