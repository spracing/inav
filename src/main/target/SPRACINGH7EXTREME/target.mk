H750xB_TARGETS += $(TARGET)

HSE_VALUE    = 8000000

FEATURES       += VCP ONBOARDFLASH

EXST = yes
EXST_ADJUST_VMA = 0x97CE0000


TARGET_SRC += \
            drivers/max7456.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
