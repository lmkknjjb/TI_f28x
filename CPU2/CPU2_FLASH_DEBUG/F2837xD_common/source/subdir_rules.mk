################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
F2837xD_common/source/F2837xD_Adc.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_Adc.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_CodeStartBranch.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_CodeStartBranch.asm $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_CpuTimers.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_CpuTimers.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_DBGIER.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_DBGIER.asm $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_DefaultISR.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_DefaultISR.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_Dma.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_Dma.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_Emif.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_Emif.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_Gpio.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_Gpio.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_I2C.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_I2C.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_Ipc.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_Ipc.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_Ipc_Driver.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_Ipc_Driver.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_Ipc_Driver_Lite.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_Ipc_Driver_Lite.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_Ipc_Driver_Util.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_Ipc_Driver_Util.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_Mcbsp.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_Mcbsp.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_PieCtrl.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_PieCtrl.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_PieVect.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_PieVect.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_Sci.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_Sci.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_Spi.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_Spi.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_SysCtrl.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_SysCtrl.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_can.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_can.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_sci_io.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_sci_io.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_sdfm_drivers.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_sdfm_drivers.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_struct.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_struct.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/source/F2837xD_usDelay.obj: C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/source/F2837xD_usDelay.asm $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/GUNACO_FCS_DC/CPU2" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/inc" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_common/include/FlashAPI/Constants" --include_path="C:/ti/GUNACO_FCS_DC/CPU1/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU2 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_common/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


