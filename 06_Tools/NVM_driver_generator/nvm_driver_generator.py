## @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
import json
import io
import time

INDENT_WIDTH = 2
ONE_INDENT = INDENT_WIDTH * " "
BLOCK_OFFSET_DECLARATION_TEMPLATE = "static constexpr uint16_t skuBlockOffset{block_name} = {value};\n"
BLOCK_OBJECT_DECLARATION_TEMPLATE = "using CNvmBlock{block_name} = TNvmBlock<S{block_name}, skuBlockOffset{block_name}>;\n"

def get_indent(indent_level):
    return ' ' * INDENT_WIDTH * indent_level

def start_object_declaration(fileObject, indentLevel, className, isStruct=False):
    indent = get_indent(indentLevel)
    
    if isStruct:
        object_type = "struct"
    else:
        object_type = "class"

    class_declaration_start = "\n" + indent + "{1} {0}\n".format(className, object_type) + indent + "{\n"
    fileObject.write(class_declaration_start)

def end_object_declaration(fileObject, indentLevel):
    indent = get_indent(indentLevel)
    class_declaration_end = indent + "};\n"
    fileObject.write(class_declaration_end)

def get_variable_prefix(variable_type):
    # As we support only native C-types, the first letter of the type name will work.
    # The function will need to be modified when struct support is implemented.
    return variable_type[0]

def declare_object_property(fileObject, indentLevel, property_params, allowed_types):
    if property_params["type"] in allowed_types:
        indent = get_indent(indentLevel)
        array_length = property_params["length"]

        if isinstance(array_length, int) and array_length > 0:
            array_prefix = "a" if array_length > 1 else ""
            array_size = "[{}]".format(array_length) if array_length > 1 else ""
            variable_name = "{}{}_".format(get_variable_prefix(property_params["type"]), property_params["name"])
            fileObject.write(indent + "{} {}{}{};\n".format(property_params["type"], array_prefix, variable_name, array_size))
        else:
            # need to raise exception
            pass
    else:
        # need to raise exception
        pass

def declare_block_offsets(outputStream, indentLevel, nvm_map):
    indent = get_indent(indentLevel)
    block_index = 0

    for block in nvm_map["blocks"]:
        if block_index == 0:
            offset_value = "0U"
        else:
            previous_block_name = nvm_map["blocks"][block_index - 1]["name"]
            offset_value = "skuBlockOffset{previous_block_name} + sizeof(S{previous_block_name})".format(previous_block_name=previous_block_name,
                                                                                                         classname=nvm_map["class_name"])

        outputStream.write(indent + BLOCK_OFFSET_DECLARATION_TEMPLATE.format(block_name=block["name"],
                                                                           value=offset_value))


        block_index = block_index + 1


def generate_declaration_of_block_types(nvm_map):
    declarationTextStream = io.StringIO("")

    for nvm_block in nvm_map["blocks"]:
        struct_name = nvm_block["name"]
        indent_level = 1
        start_object_declaration(declarationTextStream, indent_level, "S" + struct_name, True)

        for struct_field in nvm_block["fields"]:
            declare_object_property(declarationTextStream, indent_level + 1, struct_field, nvm_map["supported_types"])

        end_object_declaration(declarationTextStream, indent_level)

    return declarationTextStream.getvalue()


def generate_declaration_of_block_objects(nvm_map):
    declarationTextStream = io.StringIO("")
    indent = get_indent(1)

    for block in nvm_map["blocks"]:
        declarationTextStream.write(indent + BLOCK_OBJECT_DECLARATION_TEMPLATE.format(block_name=block["name"]))

    return declarationTextStream.getvalue()


def generate_declaration_of_block_offsets(nvm_map):
    declarationTextStream = io.StringIO("")
    declare_block_offsets(declarationTextStream, 1, nvm_map)
    return declarationTextStream.getvalue()


def generate_full_nvm_struct_declaration(nvm_map):
    declarationTextStream = io.StringIO("")
    current_indent_level = 1

    start_object_declaration(declarationTextStream, current_indent_level, nvm_map["full_nvm_struct_name"], True)

    for nvm_block in nvm_map["blocks"]:
        indent = get_indent(current_indent_level + 1)
        declarationTextStream.write(indent + "{0}{1} o{1};\n".format("S", nvm_block["name"]))

    end_object_declaration(declarationTextStream, current_indent_level)

    return declarationTextStream.getvalue()


def generate_header(nvm_map):
    with open(nvm_map["class_name"] + '.h', 'w') as headerFile:
        with open("CNvmDriver_template.h") as driverTemplate:
            driver_header = driverTemplate.read()
            driver_header = driver_header.replace("{DATE_GENERATED}", time.strftime("%d.%m.%Y at %H:%M:%S"))
            driver_header = driver_header.replace("{CLASS_NAME}", nvm_map["class_name"])
            driver_header = driver_header.replace("{INCLUSION_GUARD}", nvm_map["inclusion_guard"])
            driver_header = driver_header.replace("{MEMORY_SIZE}", "{}U".format(nvm_map["memory_size"]))
            driver_header = driver_header.replace("{VERSION_MAJOR}", "{}U".format(nvm_map["version"]["major"]))
            driver_header = driver_header.replace("{VERSION_MINOR}", "{}U".format(nvm_map["version"]["minor"]))
            driver_header = driver_header.replace("{STRUCT_ALIGNMENT}", "{}".format(nvm_map["struct_alignment"]))
            driver_header = driver_header.replace("{NVM_FULL_STRUCT_NAME}", nvm_map["full_nvm_struct_name"])
            tmp_text = generate_declaration_of_block_types(nvm_map)
            driver_header = driver_header.replace("{DECLARATION_OF_BLOCK_TYPES}", tmp_text)
            tmp_text = generate_declaration_of_block_offsets(nvm_map)
            driver_header = driver_header.replace("{DECLARATION_OF_BLOCK_OFFSETS}", tmp_text)
            tmp_text = generate_declaration_of_block_objects(nvm_map)
            driver_header = driver_header.replace("{DECLARATION_OF_BLOCK_OBJECTS}", tmp_text)
            tmp_text = generate_full_nvm_struct_declaration(nvm_map)
            driver_header = driver_header.replace("{DECLARATION_OF_FULL_NVM_STRUCT}", tmp_text)
            headerFile.write(driver_header)


def generate_nvm_driver(jsonFileName):
    with open(jsonFileName) as jsonFile:
        nvm_map = json.load(jsonFile)
        generate_header(nvm_map)

if __name__ == "__main__":
    generate_nvm_driver('nvm_map.json')