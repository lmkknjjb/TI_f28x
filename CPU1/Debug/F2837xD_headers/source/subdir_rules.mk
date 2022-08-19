################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
F2837xD_headers/source/%.obj: ../F2837xD_headers/source/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1011/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --advice:performance=all --abi=coffabi --preproc_with_compile --preproc_dependency="F2837xD_headers/source/$(basename $(<F)).d_raw" --obj_directory="F2837xD_headers/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


