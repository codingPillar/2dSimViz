OUT_NAME:=compute

SRC_FILE:=compute.cpp

CXX:=g++
FLAGS:=-Wall -Wextra
INCLUDE:=-I/opt/ros/noetic/include
LIBRARY:=-L/opt/ros/noetic/lib -L/main/bin -lpthread -lroscpp -lrosconsole -lroscpp_serialization -lrostime

define COMPILE =
$(CXX) $(FLAGS) $^ $(INCLUDE) -o $@ $(LIBRARY)
endef

$(OUT_NAME): $(SRC_FILE)
	$(COMPILE)

# COMPILE COMMANDS GENERATION
COMPILE_COMMAND_EXEC:=comp_database_gen
COMPILE_COMMANDS_FILE:=compile_commands.json
PWD:=$(shell pwd)
commands: $(SRC_FILE)
	$(shell $(COMPILE_COMMAND_EXEC) $(COMPILE_COMMANDS_FILE) $(PWD) "$(COMPILE)" $<)
	$(shell $(COMPILE_COMMAND_EXEC) $(COMPILE_COMMANDS_FILE) --end)

clean:
	rm -rf .cache
	rm -rf $(OUT_NAME)