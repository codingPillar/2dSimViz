SRC_FOLDER:=src
OUT_FOLDER:= compile

SRC:=$(wildcard $(SRC_FOLDER)/*.cpp)
SRC_OBJ:=$(SRC:$(SRC_FOLDER)/%=$(OUT_FOLDER)/%.o)

CXX:=g++
CXX_INCLUDE:=
CXX_FLAGS:=-Wall -Wextra
CXX_LINK_FLAGS:=

OUT_FILE:=node

define COMPILE =
$(CXX) $(CXX_FLAGS) $(CXX_INCLUDE) -c $< -o $@
endef

$(OUT_FOLDER)/%.o:$(SRC_FOLDER)/%
	$(COMPILE)

$(OUT_FILE):$(SRC_OBJ)
	$(CXX) $(CXX_FLAGS) $^ -o $@ $(CXX_LINK_FLAGS)

# COMPILE COMMANDS

clean:
	rm -rf $(OUT_FOLDER)/*