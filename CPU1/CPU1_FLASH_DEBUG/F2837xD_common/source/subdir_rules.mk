################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
F2837xD_common/source/%.obj: ../F2837xD_common/source/%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --fp_mode=relaxed --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU1 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/%.obj: ../F2837xD_common/source/%.asm $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --fp_mode=relaxed --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU1 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


