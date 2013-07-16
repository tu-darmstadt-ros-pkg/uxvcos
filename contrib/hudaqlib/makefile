# makefile for hudaqlib for (gcc)
# runs under Linux
#
TARGET_LIB = hudaqlib.a
TARGET_DLL = libhudaqlib.so.2.4

#optional components
MF61X   := 1
MF62X   := 1
PCI1753 := 1
PCD7004 := 1
PCT7303B := 1


GCC=gcc
FLAGS=-Wall -O2 -I./include/ -fno-exceptions
#FLAGS+=-DERROR_HANDLER
OBJ=.o
LOBJ=.lo

OBJDIR := .


OBJECTS+=$(OBJDIR)/resources$(OBJ)
OBJECTS+=$(OBJDIR)/generic$(OBJ)
OBJECTS+=$(OBJDIR)/opendevh$(OBJ)

LOBJECTS+=$(OBJDIR)/resources$(LOBJ)
LOBJECTS+=$(OBJDIR)/generic$(LOBJ)
LOBJECTS+=$(OBJDIR)/opendevh$(LOBJ)

ifneq ($(MF61X),0)
  LOBJECTS+=$(OBJDIR)/MF614$(LOBJ)
  OBJECTS+=$(OBJDIR)/MF614$(OBJ)
  FLAGS+=-DMF61X
endif
ifneq ($(MF62X),0)
  LOBJECTS+=$(OBJDIR)/MF624$(LOBJ)
  OBJECTS+=$(OBJDIR)/MF624$(OBJ)
  FLAGS+=-DMF62X
endif
ifneq ($(PCI1753),0)
  LOBJECTS+=$(OBJDIR)/PCI1753$(LOBJ)
  OBJECTS+=$(OBJDIR)/PCI1753$(OBJ)
  FLAGS+=-DPCI1753
endif
ifneq ($(PCD7004),0)
  LOBJECTS+=$(OBJDIR)/PCD7004$(LOBJ)
  OBJECTS+=$(OBJDIR)/PCD7004$(OBJ)
  FLAGS+=-DPCD7004
endif
ifneq ($(PCT7303B),0)
  LOBJECTS+=$(OBJDIR)/PCT7303B$(LOBJ)
  OBJECTS+=$(OBJDIR)/PCT7303B$(OBJ)
  FLAGS+=-DPCT7303B
endif


.SUFFIXES: .c .cc $(OBJ) $(LOBJ) .exe .a .so

.cc$(OBJ): $(MAKEFILE_LIST)
	$(GCC) $(FLAGS) $(DEBUG) -c $*.cc $(ERROR) -o $*$(OBJ) $(GETTEXT)

.c$(OBJ): $(MAKEFILE_LIST)
	$(GCC) $(FLAGS) $(DEBUG) -c $*.c $(ERROR) -o $*$(OBJ) $(GETTEXT)

.c$(LOBJ): $(MAKEFILE_LIST)
	$(GCC) $(FLAGS) -fpic $(DEBUG) -c $*.c $(ERROR) -o $*$(LOBJ) $(GETTEXT)


  
default: $(OBJDIR) $(TARGET_LIB) $(TARGET_DLL)

all:     $(OBJDIR) $(TARGET_LIB) $(TARGET_DLL)
#	cp -f strings/*.h include/*.h
#	cp -f sets/*.h include/*.h

  
$(TARGET_LIB): $(OBJECTS) $(MAKEFILE_LIST)
	echo $(OBJECTS)
#	echo $(FLAGS)
	ar rcs $(TARGET_LIB) $(OBJECTS)

$(TARGET_DLL): $(LOBJECTS) $(MAKEFILE_LIST)
	gcc -shared $(LOBJECTS) -fvisibilty=hidden -lrt -Wl,-soname,libhudaqlib.so -o $(TARGET_DLL)
	ln -s -f $(TARGET_DLL) libhudaqlib.so

$(OBJDIR) :
	mkdir -p $@


$(OBJDIR)/struct$(OBJ) $(OBJDIR)/struct$(LOBJ) :       struct.c

$(OBJDIR)/resources$(OBJ) $(OBJDIR)/resources$(LOBJ) : resources.c hudaqlib.h hudaq_internal.h
$(OBJDIR)/generic$(OBJ) $(OBJDIR)/generic$(LOBJ) :     generic.c hudaqlib.h hudaq_internal.h
$(OBJDIR)/MF614$(OBJ) $(OBJDIR)/MF614$(LOBJ) :	       MF614.c hudaqlib.h hudaq_internal.h
$(OBJDIR)/MF624$(OBJ) $(OBJDIR)/MF624$(LOBJ) :	       MF624.c hudaqlib.h hudaq_internal.h
$(OBJDIR)/opendevh$(OBJ) $(OBJDIR)/opendevh$(LOBJ) :   opendevh.c


patch:
	patch /usr/share/pci.ids ./pci.ids.patch

clean:
	rm -f *.exe bin/*.exe *$(OBJ) *$(LOBJ) *.obj *.bak core *.bak *.bkp *.a *.so
	rm -f $(TARGET_DLL) include/*.bak include/*.bkp

distclean: clean
	rm -f *.a *.so	

install: $(TARGET_DLL)
	cp $(TARGET_DLL) /lib
	ldconfig /lib



