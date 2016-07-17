AM_CFLAGS = \
   -I../../../utils \
   -I../../../platform_lib_abstractions \
   -I./rpc_inc \
   -I../libloc_api-rpc-stub/inc \
   -I../../libloc_api_50001 \
   $(MSM7K_CFLAGS) \
   -DUSE_QCOM_AUTO_RPC

requiredlibs = \
   ../../../utils/libgps_utils_so.la \
   $(MSM7K_LIBS)

h_sources = \
    rpc_inc/loc_api_rpc_glue.h \
    rpc_inc/loc_api_fixup.h \
    rpc_inc/loc_api_sync_call.h \
    rpc_inc/loc_apicb_appinit.h \

c_sources = \
    src/loc_api_rpc_glue.c \
    src/loc_api_sync_call.c \
    src/loc_apicb_appinit.c \
    src/loc_api_fixup.c \
    src/loc_api_log.c \
    src/LocApiRpcAdapter.cpp \


library_includedir = $(pkgincludedir)/libloc_api-rpc-50001/libloc_api-rpc-glue/rpc_inc
library_include_HEADERS = $(h_sources)
libloc_api_rpc_qc_la_SOURCES = $(c_sources) $(h_sources)

if USE_GLIB
libloc_api_rpc_qc_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
libloc_api_rpc_qc_la_LDFLAGS = -lstdc++ -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libloc_api_rpc_qc_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
libloc_api_rpc_qc_la_CFLAGS = $(AM_CFLAGS)
libloc_api_rpc_qc_la_LDFLAGS = -lpthread -shared -version-info 1:0:0
libloc_api_rpc_qc_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

libloc_api_rpc_qc_la_LIBADD = $(requiredlibs) -lstdc++

#Create and Install Libraries
lib_LTLIBRARIES = libloc_api_rpc_qc.la
