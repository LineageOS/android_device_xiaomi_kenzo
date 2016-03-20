AM_CFLAGS = -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs
AM_CFLAGS += -I./../inc
#AM_CFLAGS += -DDEBUG -g

common_CFLAGS =  -DUSE_GLIB @GLIB_CFLAGS@
common_LDFLAGS = -lrt @GLIB_LIBS@

c_sources   = ipa_nat_drv.c \
              ipa_nat_drvi.c \
              ipa_nat_logi.c

library_includedir = $(pkgincludedir)
library_include_HEADERS = ./../inc/ipa_nat_drvi.h \
                          ./../inc/ipa_nat_drv.h \
                          ./../inc/ipa_nat_logi.h

lib_LTLIBRARIES = libipanat.la
libipanat_la_C = @C@
libipanat_la_SOURCES = $(c_sources)
libipanat_la_CFLAGS = $(AM_CFLAGS) $(common_CFLAGS)
libipanat_la_LDFLAGS = -shared $(common_LDFLAGS) -version-info 1:0:0
