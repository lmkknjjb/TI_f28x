################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
F2837xD_common/source/%.obj: ../F2837xD_common/source/%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs901/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --fp_mode=relaxed --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/source" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/lib" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/source" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/source" --include_path="C:/ti/ccs901/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1" --advice:performance=all --define=CPU1 --define=_FLASH --define=_STANDALONE --abi=coffabi --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/%.obj: ../F2837xD_common/source/%.asm $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs901/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --fp_mode=relaxed --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/source" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/lib" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/source" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/source" --include_path="C:/ti/ccs901/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1" --advice:performance=all --define=CPU1 --define=_FLASH --define=_STANDALONE --abi=coffabi --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


