## @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
import json
import io
import time
import sys
import os
import argparse

INDENT_WIDTH = 2
ONE_INDENT = INDENT_WIDTH * " "
PORT_DECLARATION_TEMPLATE = "CSoftwareComponentPort<{type}, static_cast<uint8_t>(EPortIds::e{name}), {timeout_ms}> oPort{name}_;\n"
PORT_INITIALIZATION_TEMPLATE = "oPort{name}_.Init();\n";
PORT_SET_RECORD_FOR_DEBUG_TEMPLATE = "oPort{name}_.SetRecordForDebug(true);\n";
PORT_ID_LABEL_TEMPLATE = "e{name}"
FRIEND_COMPONENT_DECLARATION_TEMPLATE = "friend {class_name};\n"
INCLUSION_OF_COMPONENT_HEADER_TEMPLATE = "#include \"{header_file}\"\n"
TO_BYTE_VECTOR_FUNCTION ="""
std::vector<uint8_t> ToByteVector() final
{
{INDENT}std::vector<uint8_t> oBytes;
{INDENT}bool bStatus =  NLibCommon::Serialize(oBytes, {LIST_OF_STRUCT_FIELDS});

{INDENT}if (false == bStatus)
{INDENT}{
{INDENT}{INDENT}oBytes.clear();
{INDENT}}

{INDENT}return oBytes;
}
"""

FROM_BYTE_VECTOR_FUNCTION ="""
bool FromByteVector(const std::vector<uint8_t>& korBytes) final
{
{INDENT}bool bStatus = NLibCommon::Deserialize(korBytes, {LIST_OF_STRUCT_FIELDS});

{INDENT}if (false == bStatus)
{INDENT}{
{INDENT}{INDENT}*this = {PORT_DATA_TYPE_NAME}();
{INDENT}}

{INDENT}return bStatus;
}
"""

PORT_DATA_VECTOR_DECLARATION = "static vector<S{type}> oPortData{name}Container;\n"

PORT_ID_SWITCH = """
case CRte::EPortIds::e{name}:
{INDENT}{
{INDENT}{INDENT}S{type} oPortData;

{INDENT}{INDENT}if (oPortData.FromByteVector(oBytes))
{INDENT}{INDENT}{
{INDENT}{INDENT}{INDENT}oPortData{name}Container.push_back(oPortData);
{INDENT}{INDENT}}
{INDENT}}
{INDENT}break;
"""

PORT_DATA_WRITER_DECLARATION = "static void writePort{name}ToMat(TinyMATWriterFile* opMatWriter, std::vector<S{type}>& orPortDataContainer);\n"

PORT_DATA_WRITER_FUNCTION = """
void writePort{name}ToMat(TinyMATWriterFile* opMatWriter, std::vector<S{type}>& orPortDataContainer)
{
{DECLARE_POINTERS}
  try
  {
{RESERVE_MEMORY_FOR_SCALAR_ARRAYS}

    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
{ASSIGN_DATA_TO_SCALAR_ARRAYS}
    }

    TinyMATWriter_startStruct(opMatWriter, "Port{name}");
{WRITE_STRUCT_FIELDS_AS_ROWS}
    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct Port{name}..." << endl;
  }

{FREE_STORAGE}
}
"""

WRITE_TO_PORT_BY_ID_SWITCH = """
case CRte::EPortIds::e{name}:
{INDENT}{
{INDENT}{INDENT}S{type} oPortData;

{INDENT}{INDENT}if (oPortData.FromByteVector(korBytes))
{INDENT}{INDENT}{
{INDENT}{INDENT}{INDENT}CRte::GetInstance().oPort{name}_.Write(oPortData);
{INDENT}{INDENT}}
{INDENT}}
{INDENT}break;
"""
UINT32_MAX = 0xFFFFFFFF

def get_indent(indent_level):
    return ' ' * INDENT_WIDTH * indent_level

def get_variable_prefix(variable_type):
    # As we support only native C-types, the first letter of the type name will work.
    # The function will need to be modified when struct support is implemented.
    return variable_type[0]

def get_variable_name(variable_type, variable_name, variable_length):
    return "{}{}{}_".format("a" if variable_length > 1 else "", get_variable_prefix(variable_type), variable_name)

def get_default_value_in_braces(variable_type, default_value, array_length):
    if variable_type == "char":
        value = "'" + default_value + "'"
    elif variable_type == "float":
        value = "{}F".format(default_value)
    elif variable_type == "uint8_t" or variable_type == "uint16_t" or variable_type == "uint32_t" or variable_type == "uint64_t":
        value = "{}U".format(default_value)
    else:
        value = "{}".format(default_value)

    textStream = io.StringIO("")
    textStream.write("{ ")

    if array_length > 0:
        for ind in range(array_length):
            textStream.write(value)
            if ind < (array_length - 1):
                textStream.write(", ")
    else:
        sys.exit("Error: array length must be positive.")

    textStream.write(" }")

    return textStream.getvalue()

def is_non_negative_integer(value):
    return (not isinstance(value, bool)) and isinstance(value, int) and (value >= 0)

def validate_timeout_ms(timeout_ms, timeout_context):
    if not is_non_negative_integer(timeout_ms):
        sys.exit("Error: " + timeout_context + " must be a non-negative integer timeout in milliseconds.")

    if timeout_ms > UINT32_MAX:
        sys.exit("Error: " + timeout_context + " must not exceed " + str(UINT32_MAX) + ".")

def get_timeout_ms_literal(timeout_ms):
    return "{}U".format(timeout_ms)

def get_default_port_invalidation_timeout_ms(rte_config):
    timeout_ms = 0

    if "default_port_invalidation_timeout_ms" in rte_config:
        timeout_ms = rte_config["default_port_invalidation_timeout_ms"]

    return timeout_ms

def get_port_invalidation_timeout_ms(rte_config, port):
    timeout_ms = get_default_port_invalidation_timeout_ms(rte_config)

    if "invalidation_timeout_ms" in port:
        timeout_ms = port["invalidation_timeout_ms"]

    return timeout_ms

def declare_object_property(fileObject, indentLevel, property_params, allowed_types):
    if property_params["type"] in allowed_types:
        indent = get_indent(indentLevel)
        array_length = property_params["length"]

        if isinstance(array_length, int) and array_length > 0:
            array_size = "[{}]".format(array_length) if array_length > 1 else ""
            variable_name = get_variable_name(property_params["type"], property_params["name"], property_params["length"])
            default_value = get_default_value_in_braces(property_params["type"], property_params["default_value"], property_params["length"])
            fileObject.write(indent + "{} {}{}{}; ///< {}\n".format(property_params["type"], variable_name, array_size, default_value, property_params["description"]))
        else:
            # need to raise exception
            pass
    else:
        # need to raise exception
        pass

def generate_declaration_of_ports(rte_config):
    declarationTextStream = io.StringIO("")
    indent = get_indent(1)

    for port in rte_config["ports"]:
        if port["generated"]:
            port_type = "S" + port["type"]
        else:
            port_type = port["type"]

        timeout_ms = get_port_invalidation_timeout_ms(rte_config, port)
        declarationTextStream.write(indent + PORT_DECLARATION_TEMPLATE.format(name=port["name"], type=port_type, timeout_ms=get_timeout_ms_literal(timeout_ms)))

    return declarationTextStream.getvalue()

def generate_declaration_of_port_ids(rte_config):
    declarationTextStream = io.StringIO("")
    indent = get_indent(1)

    declarationTextStream.write(indent + "enum class EPortIds : uint8_t\n" + indent + "{\n");

    portIndex = 0

    for port in rte_config["ports"]:        
        declarationTextStream.write(indent + indent + PORT_ID_LABEL_TEMPLATE.format(name=port["name"]))

        if portIndex != (len(rte_config["ports"]) - 1):
            declarationTextStream.write(",")

        declarationTextStream.write("\n")
        portIndex += 1

    declarationTextStream.write(indent + "};\n");

    return declarationTextStream.getvalue()

def generate_declaration_of_components(rte_config):
    declarationTextStream = io.StringIO("")
    indent = get_indent(1)

    has_component_not_included_in_resimulation = False

    # At first declare components that are present both in the target project and in the resimulation
    for component in rte_config["components"]:
        if component["include_in_resimulation"]:
            declarationTextStream.write(indent + FRIEND_COMPONENT_DECLARATION_TEMPLATE.format(class_name=component["class_name"]))
        else:
            has_component_not_included_in_resimulation = True

    if has_component_not_included_in_resimulation:
        declarationTextStream.write("#ifndef _MSC_VER\n")

        for component in rte_config["components"]:
            if not component["include_in_resimulation"]:
                declarationTextStream.write(indent + FRIEND_COMPONENT_DECLARATION_TEMPLATE.format(class_name=component["class_name"]))

        declarationTextStream.write("#endif /* _MSC_VER */\n")

    return declarationTextStream.getvalue()

def generate_declaration_of_component_header_inclusions(rte_config):
    declarationTextStream = io.StringIO("")
    emitted_header_files = set()
    non_resimulation_header_files = []

    # At first declare components that are present both in the target project and in the resimulation
    for component in rte_config["components"]:
        header_file = component["header_file"]

        if component["include_in_resimulation"]:
            if header_file not in emitted_header_files:
                declarationTextStream.write(INCLUSION_OF_COMPONENT_HEADER_TEMPLATE.format(header_file=header_file))
                emitted_header_files.add(header_file)
        else:
            if (header_file not in emitted_header_files) and (header_file not in non_resimulation_header_files):
                non_resimulation_header_files.append(header_file)

    if len(non_resimulation_header_files) > 0:
        declarationTextStream.write("#ifndef _MSC_VER\n")

        for header_file in non_resimulation_header_files:
            declarationTextStream.write(INCLUSION_OF_COMPONENT_HEADER_TEMPLATE.format(header_file=header_file))

        declarationTextStream.write("#endif /* _MSC_VER */\n")

    return declarationTextStream.getvalue()

def generate_initialization_of_ports(rte_config):
    declarationTextStream = io.StringIO("")
    indent = get_indent(1)

    for port in rte_config["ports"]:        
        declarationTextStream.write(indent + PORT_INITIALIZATION_TEMPLATE.format(name=port["name"]))

    return declarationTextStream.getvalue()

def generate_default_debug_output(rte_config):
    textStream = io.StringIO("")
    indent = get_indent(1)

    textStream.write("#ifdef SEND_DEBUG_OUTPUT\n")

    for port in rte_config["ports"]:
        if port["record_by_default"]:
            textStream.write(indent + PORT_SET_RECORD_FOR_DEBUG_TEMPLATE.format(name=port["name"]))

    textStream.write("#endif /* SEND_DEBUG_OUTPUT */\n")

    return textStream.getvalue()

def generate_declaration_of_port_data_types(rte_config):
    declarationTextStream = io.StringIO("")
    
    for port in rte_config["port_data_types"]:
        # Start struct declaration
        declarationTextStream.write("struct S{} final : public NLibCommon::CSerializable\n{{\n".format(port["name"]))

        # Declare struct fields
        for struct_field in port["fields"]:
            declare_object_property(declarationTextStream, 1, struct_field, rte_config["supported_port_field_types"])

        # Generate serialization APIs
        indent = get_indent(1)
        fieldListTextStream = io.StringIO("")

        # Generate comma-separated variable list
        fieldCount = len(port["fields"])
        fieldIndex = 0

        for struct_field in port["fields"]:
            if struct_field["length"] == 1:
                fieldListTextStream.write("\n{INDENT}{INDENT}" + get_variable_name(struct_field["type"], struct_field["name"], struct_field["length"]))
            elif struct_field["length"] > 1:
                for ind in range(struct_field["length"]):
                    fieldListTextStream.write("\n{INDENT}{INDENT}" + get_variable_name(struct_field["type"], struct_field["name"], struct_field["length"]) + "[{}]".format(ind))

                    if ind < (struct_field["length"] - 1):
                        fieldListTextStream.write(",")
            else:
                sys.exit("Error: array length must be positive in port " + port["name"] + " field " + struct_field["name"] + ".")

            
            if fieldIndex != (fieldCount - 1):
                fieldListTextStream.write(",")

            fieldIndex += 1

        # Generate ToByteVector()
        tmp_text = TO_BYTE_VECTOR_FUNCTION.replace("{LIST_OF_STRUCT_FIELDS}", fieldListTextStream.getvalue())
        tmp_text = tmp_text.replace("{INDENT}", ONE_INDENT)
        lines_tmp = tmp_text.splitlines(True)

        declarationTextStream.write("\n#if defined(SEND_DEBUG_OUTPUT) || defined(_MSC_VER)\n")

        for single_line in lines_tmp:
            declarationTextStream.write(indent + single_line)

        # Generate FromByteVector()
        tmp_text = FROM_BYTE_VECTOR_FUNCTION.replace("{LIST_OF_STRUCT_FIELDS}", fieldListTextStream.getvalue())
        tmp_text = tmp_text.replace("{PORT_DATA_TYPE_NAME}", "S" + port["name"])
        tmp_text = tmp_text.replace("{INDENT}", ONE_INDENT)
        lines_tmp = tmp_text.splitlines(True)

        for single_line in lines_tmp:
            declarationTextStream.write(indent + single_line)

        declarationTextStream.write("#endif /* SEND_DEBUG_OUTPUT || _MSC_VER */\n")

        #End struct declaration
        declarationTextStream.write("};\n\n")

    return declarationTextStream.getvalue()

def generate_declaration_of_port_data_vectors(rte_config):
    declarationTextStream = io.StringIO("")

    for port in rte_config["ports"]:
        if port["generated"]:
            declarationTextStream.write(PORT_DATA_VECTOR_DECLARATION.format(name=port["name"], type=port["type"]))

    return declarationTextStream.getvalue()

def generate_port_id_switch(rte_config):
    textStream = io.StringIO("")
    indent = get_indent(2)

    for port in rte_config["ports"]:
        if port["generated"]:
            tmp_text = PORT_ID_SWITCH.replace("{INDENT}", ONE_INDENT)
            tmp_text = tmp_text.replace("{name}", port["name"])
            tmp_text = tmp_text.replace("{type}", port["type"])
            lines_tmp = tmp_text.splitlines(True)

            for single_line in lines_tmp:
                textStream.write(indent + single_line)

    return textStream.getvalue()

def generate_declarations_of_mat_writers(rte_config):
    declarationTextStream = io.StringIO("")

    for port in rte_config["ports"]:
        if port["generated"]:
            declarationTextStream.write(PORT_DATA_WRITER_DECLARATION.format(name=port["name"], type=port["type"]))

    return declarationTextStream.getvalue()

def generate_write_port_data_to_mat(rte_config):
    textStream = io.StringIO("")
    indent = get_indent(2)

    for port in rte_config["ports"]:
        if port["generated"]: 
            textStream.write(indent + "writePort{name}ToMat(opMat, oPortData{name}Container);\n".format(name=port["name"]))

    return textStream.getvalue()

def generate_implementations_of_mat_writers(rte_config):
    textStream = io.StringIO("")

    for port in rte_config["ports"]:
        if port["generated"]:
            tmp_text = PORT_DATA_WRITER_FUNCTION.replace("{name}", port["name"])
            tmp_text = tmp_text.replace("{type}", port["type"])

            declarePointersStream = io.StringIO("")
            reserveMemoryStream = io.StringIO("")
            freeMemoryStream = io.StringIO("")
            assignFieldsStream = io.StringIO("")
            writeFieldsStream = io.StringIO("")

            indent = get_indent(2)

            for data_type in rte_config["port_data_types"]:
                if data_type["name"] == port["type"]:
                    for struct_field in data_type["fields"]:
                        declarePointersStream.write(get_indent(1) + "{type}* {prefix}p{field_name} {{nullptr}};\n".format(type=struct_field["type"], prefix=get_variable_prefix(struct_field["type"]), field_name=struct_field["name"]))
                        freeMemoryStream.write(get_indent(1) + "delete [] {prefix}p{field_name};\n".format(prefix=get_variable_prefix(struct_field["type"]), field_name=struct_field["name"]))

                        reserveMemoryStream.write(get_indent(2) + "{prefix}p{field_name} = new {type}[orPortDataContainer.size(){multiplicator}];\n".format( \
                            type=struct_field["type"], \
                            prefix=get_variable_prefix(struct_field["type"]), \
                            field_name=struct_field["name"], \
                            multiplicator=" * {}".format(struct_field["length"]) if struct_field["length"] > 1 else ""))

                        if struct_field["length"] == 1:
                            assignFieldsStream.write(get_indent(3) + "{prefix}p{field_name}[uIndex] = orPortDataContainer[uIndex].{variable_name};\n".format( \
                                prefix=get_variable_prefix(struct_field["type"]), \
                                field_name=struct_field["name"], \
                                variable_name=get_variable_name(struct_field["type"], struct_field["name"], struct_field["length"])))
                        elif struct_field["length"] > 1:
                            assignFieldsStream.write("\n")
                            assignFieldsStream.write(get_indent(3) +"for (uint32_t uInd = 0U; uInd < {}; ++uInd)\n".format(struct_field["length"]))
                            assignFieldsStream.write(get_indent(3) +"{\n")

                            assignFieldsStream.write(get_indent(4) + "{prefix}p{field_name}[uInd * orPortDataContainer.size() + uIndex] = orPortDataContainer[uIndex].{variable_name}[uInd];\n".format( \
                                prefix=get_variable_prefix(struct_field["type"]), \
                                field_name=struct_field["name"], \
                                variable_name=get_variable_name(struct_field["type"], struct_field["name"], struct_field["length"])))

                            assignFieldsStream.write(get_indent(3) +"}\n\n")

                        if struct_field["length"] == 1:
                            writeFieldsStream.write(get_indent(2) + 'TinyMATWriter_writeVectorAsRow(opMatWriter, "{variable_name}", {prefix}p{field_name}, static_cast<int32_t>(orPortDataContainer.size()));\n'.format( \
                                prefix=get_variable_prefix(struct_field["type"]), \
                                field_name=struct_field["name"], \
                                variable_name=get_variable_name(struct_field["type"], struct_field["name"], struct_field["length"])))
                        elif struct_field["length"] > 1:
                            writeFieldsStream.write(get_indent(2) + 'TinyMATWriter_writeMatrix2D_rowmajor(opMatWriter, "{variable_name}", {prefix}p{field_name}, static_cast<int32_t>(orPortDataContainer.size()), 3);\n'.format( \
                                prefix=get_variable_prefix(struct_field["type"]), \
                                field_name=struct_field["name"], \
                                variable_name=get_variable_name(struct_field["type"], struct_field["name"], struct_field["length"])))

            tmp_text = tmp_text.replace("{DECLARE_POINTERS}", declarePointersStream.getvalue())
            tmp_text = tmp_text.replace("{RESERVE_MEMORY_FOR_SCALAR_ARRAYS}", reserveMemoryStream.getvalue())
            tmp_text = tmp_text.replace("{ASSIGN_DATA_TO_SCALAR_ARRAYS}", assignFieldsStream.getvalue())
            tmp_text = tmp_text.replace("{FREE_STORAGE}", freeMemoryStream.getvalue())
            tmp_text = tmp_text.replace("{WRITE_STRUCT_FIELDS_AS_ROWS}", writeFieldsStream.getvalue())

            textStream.write(tmp_text)

    return textStream.getvalue()

def generate_write_to_port_by_id(rte_config):
    textStream = io.StringIO("")
    indent = get_indent(2)

    for port in rte_config["ports"]:
        if port["generated"]:
            tmp_text = WRITE_TO_PORT_BY_ID_SWITCH.replace("{INDENT}", ONE_INDENT)
            tmp_text = tmp_text.replace("{name}", port["name"])
            tmp_text = tmp_text.replace("{type}", port["type"])
            lines_tmp = tmp_text.splitlines(True)

            for single_line in lines_tmp:
                textStream.write(indent + single_line)

    return textStream.getvalue()

def generate_rte_header(rte_config):
    status = False
    header_path = os.path.join(rte_config["rte_path"], 'Inc', rte_config["class_name"] + '.h')
    template_file = os.path.join(rte_config["template_path"], "CRte_header_template.h")

    with open(header_path, 'w') as headerFile:
        with open(template_file) as rteTemplate:
            rte_header = rteTemplate.read()
            rte_header = rte_header.replace("{DATE_GENERATED}", time.strftime("%d.%m.%Y at %H:%M:%S"))
            rte_header = rte_header.replace("{CLASS_NAME}", rte_config["class_name"])
            rte_header = rte_header.replace("{INCLUSION_GUARD}", rte_config["inclusion_guard"])
            rte_header = rte_header.replace("{DECLARATION_OF_PORTS}", generate_declaration_of_ports(rte_config))
            rte_header = rte_header.replace("{DECLARATION_OF_SOFTWARE_COMPONENTS}", generate_declaration_of_components(rte_config))
            rte_header = rte_header.replace("{INCLUSION_OF_SOFTWARE_COMPONENT_HEADERS}", generate_declaration_of_component_header_inclusions(rte_config))
            rte_header = rte_header.replace("{DECLARATION_OF_PORT_IDS}", generate_declaration_of_port_ids(rte_config))
            headerFile.write(rte_header)
            status = True

    return status

def generate_rte_cpp(rte_config):
    status = False
    cpp_path = os.path.join(rte_config["rte_path"], 'Src', rte_config["class_name"] + '.cpp')
    template_file = os.path.join(rte_config["template_path"], "CRte_cpp_template.cpp")

    with open(cpp_path, 'w') as cppFile:
        with open(template_file) as rteTemplate:
            rte_cpp = rteTemplate.read()
            rte_cpp = rte_cpp.replace("{DATE_GENERATED}", time.strftime("%d.%m.%Y at %H:%M:%S"))
            rte_cpp = rte_cpp.replace("{CLASS_NAME}", rte_config["class_name"])
            rte_cpp = rte_cpp.replace("{INITIALIZATION_OF_PORTS}", generate_initialization_of_ports(rte_config))
            rte_cpp = rte_cpp.replace("{DEFAULT_DEBUG_OUTPUT}", generate_default_debug_output(rte_config))
            cppFile.write(rte_cpp)
            status = True
    return status

def generate_rte_types_header(rte_config):
    status = False
    header_path = os.path.join(rte_config["rte_path"], 'Inc', 'RteTypesGenerated.h')
    template_file = os.path.join(rte_config["template_path"], "RteTypes_header_template.h")

    with open(header_path, 'w') as rteTypesHeaderFile:
        with open(template_file) as rteTypesHeaderTemplate:
            rte_types_header = rteTypesHeaderTemplate.read()
            rte_types_header = rte_types_header.replace("{DATE_GENERATED}", time.strftime("%d.%m.%Y at %H:%M:%S"))
            rte_types_header = rte_types_header.replace("{DECLARATION_OF_PORT_DATA_TYPES}", generate_declaration_of_port_data_types(rte_config))
            rteTypesHeaderFile.write(rte_types_header)
            status = True
    return status

def generate_debug_output_to_mat_cpp(rte_config):
    status = False
    cpp_path = os.path.join(rte_config["debug_output_path"], 'DebugOutputToMat.cpp')
    template_file = os.path.join(rte_config["template_path"], "DebugOutputToMat_cpp_template.cpp")

    with open(cpp_path, 'w') as converterSourceFile:
        with open(template_file) as converterSourceTemplate:
            converter_cpp = converterSourceTemplate.read()
            converter_cpp = converter_cpp.replace("{DATE_GENERATED}", time.strftime("%d.%m.%Y at %H:%M:%S"))
            converter_cpp = converter_cpp.replace("{DECLARATION_OF_PORT_DATA_VECTORS}", generate_declaration_of_port_data_vectors(rte_config))
            converter_cpp = converter_cpp.replace("{PORT_ID_SWITCH}", generate_port_id_switch(rte_config))
            converter_cpp = converter_cpp.replace("{DECLARATIONS_OF_MAT_WRITERS}", generate_declarations_of_mat_writers(rte_config))
            converter_cpp = converter_cpp.replace("{WRITE_PORT_DATA_TO_MAT}", generate_write_port_data_to_mat(rte_config))
            converter_cpp = converter_cpp.replace("{IMPLEMENTATIONS_OF_PORT_DATA_WRITERS}", generate_implementations_of_mat_writers(rte_config))
            converterSourceFile.write(converter_cpp)
            status = True
    return status

def generate_write_to_port_for_resim_cpp(rte_config):
    status = False
    cpp_path = os.path.join(rte_config["replay_tool_path"], 'WriteToRtePort.cpp')
    template_file = os.path.join(rte_config["template_path"], "WriteToRtePort_template.cpp")

    with open(cpp_path, 'w') as writeToRtePortCpp:
        with open(template_file) as writeToRtePortCppTemplate:
            write_to_port_cpp = writeToRtePortCppTemplate.read()
            write_to_port_cpp = write_to_port_cpp.replace("{DATE_GENERATED}", time.strftime("%d.%m.%Y at %H:%M:%S"))
            write_to_port_cpp = write_to_port_cpp.replace("{SWITCH_WRITE_TO_PORT_BY_ID}", generate_write_to_port_by_id(rte_config))
            writeToRtePortCpp.write(write_to_port_cpp)
            status = True
    return status

def check_rte_configuration(rte_config):
    validate_timeout_ms(get_default_port_invalidation_timeout_ms(rte_config), "default_port_invalidation_timeout_ms")

    # Check that data type names are unique
    port_types = rte_config["port_data_types"]

    for indFirst in range(len(rte_config["port_data_types"])):
        for indSecond in range(indFirst + 1, len(rte_config["port_data_types"])):
            if port_types[indFirst]["name"] == port_types[indSecond]["name"]:
                sys.exit("Error: duplicated port data type name " + port_types[indFirst]["name"] + ".")

    # Check that for every port with the "generated" property equal to True
    # there is a port data type definition
    for port in rte_config["ports"]:
        validate_timeout_ms(get_port_invalidation_timeout_ms(rte_config, port), "invalidation_timeout_ms for port " + port["name"])

        if port["generated"]:
            type_declaration_found = False
            for port_type in rte_config["port_data_types"]:
                if port_type["name"] == port["type"]:
                    type_declaration_found = True
                    break

            if not type_declaration_found:
                sys.exit("Error: port data type declaration " + port["type"] + " not found for the SWC port " + port["name"] + ".")
    return

def generate_rte_from_config(rte_config):
    check_rte_configuration(rte_config)

    if generate_rte_header(rte_config):
        if generate_rte_cpp(rte_config):
            if generate_rte_types_header(rte_config):
                if generate_debug_output_to_mat_cpp(rte_config):
                    generate_write_to_port_for_resim_cpp(rte_config)

def main():
    parser = argparse.ArgumentParser(description="Generate RTE source files.")
    parser.add_argument("json_config", help="Path to rte.json configuration file.")
    parser.add_argument("--rte-path", required=True, help="Output path for RTE source/header files.")
    parser.add_argument("--debug-output-path", required=True, help="Output path for DebugOutputToMat files.")
    parser.add_argument("--replay-tool-path", required=True, help="Output path for BahrsReplayTool files.")
    args = parser.parse_args()

    # Make paths absolute and ensure directories exist
    args.rte_path = os.path.abspath(args.rte_path)
    args.debug_output_path = os.path.abspath(args.debug_output_path)
    args.replay_tool_path = os.path.abspath(args.replay_tool_path)

    os.makedirs(os.path.join(args.rte_path, 'Inc'), exist_ok=True)
    os.makedirs(os.path.join(args.rte_path, 'Src'), exist_ok=True)
    os.makedirs(args.debug_output_path, exist_ok=True)
    os.makedirs(args.replay_tool_path, exist_ok=True)

    # Load JSON config and inject paths
    with open(args.json_config) as f:
        rte_config = json.load(f)

    rte_config["rte_path"] = args.rte_path
    rte_config["debug_output_path"] = args.debug_output_path
    rte_config["replay_tool_path"] = args.replay_tool_path
    rte_config["template_path"] = os.path.dirname(os.path.abspath(__file__))

    generate_rte_from_config(rte_config)
    
    print("[INFO] RTE generation completed.\n")


if __name__ == "__main__":
    main()
