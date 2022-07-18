################################################################################
#
# people_count
#
################################################################################
PEOPLE_COUNT_LOCAL_PATH:= $(realpath $(dir $(lastword $(MAKEFILE_LIST))))
PEOPLE_COUNT_SITE = $(PEOPLE_COUNT_LOCAL_PATH)/src
PEOPLE_COUNT_SITE_METHOD = local
KMODEL=$(PEOPLE_COUNT_LOCAL_PATH)/kmodel/
PEOPLE_COUNT_CXXFLAGS = $(TARGET_CXXFLAGS) 
PEOPLE_COUNT_CFLAGS = $(TARGET_CFLAGS) 

PEOPLE_COUNT_CONF_OPTS += \
        -DCMAKE_CXX_FLAGS="$(PEOPLE_COUNT_LOCAL_PATH_CXXFLAGS) -I$(STAGING_DIR)/usr/include/opencv4" \
        -DCMAKE_C_FLAGS="$(PEOPLE_COUNT_LOCAL_PATH_CFLAGS) -I$(STAGING_DIR)/usr/include/opencv4 -I$(STAGING_DIR)/usr/include" \
	-DCMAKE_INSTALL_PREFIX="/app/people_count"


PEOPLE_COUNT_DEPENDENCIES += mediactl_lib nncase_linux_runtime opencv4 libdrm

define PEOPLE_COUNT_COMPILE_MODEL
	mkdir -p $(TARGET_DIR)/app/people_count/kmodel/
	cp $(KMODEL)/yolov5s_320_sigmoid_bf16_with_preprocess_uint8_NHWC.kmodel $(TARGET_DIR)/app/people_count/kmodel/
endef

PEOPLE_COUNT_POST_INSTALL_TARGET_HOOKS += PEOPLE_COUNT_COMPILE_MODEL

$(eval $(cmake-package))
