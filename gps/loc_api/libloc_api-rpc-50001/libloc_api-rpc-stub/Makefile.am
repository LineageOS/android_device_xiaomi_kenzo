AM_CFLAGS = \
   -I../../../utils \
   -I./inc \
   $(MSM7K_CFLAGS)

requiredlibs = \
    ../../../utils/libgps_utils_so.la \
    $(MSM7K_LIBS)

h_sources = \
    inc/loc_api_rpcgen_rpc.h \
    inc/loc_api_rpcgen_common_rpc.h \
    inc/loc_api_rpcgen_cb_rpc.h \
    inc/loc_apicb_appinit.h

c_sources = \
    src/loc_api_rpcgen_cb_xdr.c \
    src/loc_api_rpcgen_common_xdr.c \
    src/loc_api_rpcgen_cb_svc.c \
    src/loc_api_rpcgen_clnt.c \
    src/loc_api_rpcgen_xdr.c



library_includedir = $(pkgincludedir)/libloc_api-rpc-50001/libloc_api-rpc-stub/inc
library_include_HEADERS = $(h_sources)
libloc_api_rpcgen_la_SOURCES = $(c_sources) $(h_sources)

if USE_GLIB
libloc_api_rpcgen_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
libloc_api_rpcgen_la_LDFLAGS = -lstdc++ -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libloc_api_rpcgen_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
libloc_api_rpcgen_la_CFLAGS = $(AM_CFLAGS)
libloc_api_rpcgen_la_LDFLAGS = -lpthread -shared -version-info 1:0:0
libloc_api_rpcgen_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

libloc_api_rpcgen_la_LIBADD = $(requiredlibs) -lstdc++

#Create and Install Libraries
lib_LTLIBRARIES = libloc_api_rpcgen.la
