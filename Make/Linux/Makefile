MARE := ../../Util/SimRobot/Util/mare/Linux/bin/mare
CONFIG ?= Develop
SSSE3 = `(grep ssse3 </proc/cpuinfo ; echo 'ssse3=false') | head -1 | sed 's/.*ssse3 .*/ssse3=true/'`
AVX2 = `(grep avx2 </proc/cpuinfo ; echo 'avx2=false') | head -1 | sed 's/.*avx2 .*/avx2=true/'`
FLAGS = $(SSSE3) $(AVX2)

OS_VERS = $(word 1, $(subst ., , `$(shell lsb_release -a 2>/dev/null | grep Release | awk '{ print $$2 }')`))

ifeq ($(OS_VERS),`18)
QTINCLUDE = `find /usr/include /usr/local/include -name qt5 -type d`
else
QTINCLUDE = `qtchooser -qt=qt5-x86_64-linux-gnu -print-env | grep QTTOOLDIR | sed "s%QTTOOLDIR=\"\(.*\)bin\"%\1%" | sed "s/^\/usr\/lib/\/usr\/include/"`
endif

.PHONY: all clean

all:
	@$(MARE) config=$(CONFIG) qtinclude=$(QTINCLUDE) $(FLAGS)

clean:
	@$(MARE) clean config=$(CONFIG)

%:
	@$(MARE) $@ config=$(CONFIG) qtinclude=$(QTINCLUDE) $(FLAGS)

