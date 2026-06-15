var index =
[
    [ "Introduction and goals", "page_intro_and_goals.html", null ],
    [ "Architecture constraints", "page_architecture_constraints.html", null ],
    [ "System scope and context", "page_system_scope_and_context.html", null ],
    [ "Solution strategy", "page_solution_strategy.html", [
      [ "Source code version identifier", "index.html#autotoc_md0", null ],
      [ "Software architecture document chapters", "index.html#autotoc_md1", null ],
      [ "General", "page_solution_strategy.html#sec_solution_strategy_general", [
        [ "Operating system", "page_solution_strategy.html#subsec_solution_strategy_operating_system", null ],
        [ "Programming language", "page_solution_strategy.html#subsec_solution_strategy_programming_language", null ],
        [ "Source code classification", "page_solution_strategy.html#subsec_solution_strategy_source_file_types", [
          [ "Software components", "page_solution_strategy.html#subsubsec_solution_strategy_software_component", null ],
          [ "Operating system", "page_solution_strategy.html#subsubsec_solution_strategy_operating_system", null ],
          [ "Libraries", "page_solution_strategy.html#subsubsec_solution_strategy_libraries", null ]
        ] ],
        [ "Usage of dynamic memory", "page_solution_strategy.html#subsubsec_solution_strategy_dynamic_memory", [
          [ "Exceptions", "page_solution_strategy.html#subsubsec_solution_strategy_dynamic_memory_exceptions", null ],
          [ "Implementation", "page_solution_strategy.html#subsubsec_solution_strategy_dynamic_memory_implementation", null ],
          [ "Developer guidelines", "page_solution_strategy.html#subsubsec_solution_strategy_dynamic_memory_guidelines", null ]
        ] ]
      ] ],
      [ "Patterns and principles", "page_solution_strategy.html#sec_solution_strategy_patterns_and_principles", [
        [ "Software components", "page_solution_strategy.html#subsec_solution_strategy_software_components", [
          [ "Principles", "page_solution_strategy.html#subsubsec_solution_strategy_swc_principles", null ],
          [ "Design patterns", "page_solution_strategy.html#subsubsec_solution_strategy_swc_patterns", null ]
        ] ],
        [ "Runtime environment (RTE)", "page_solution_strategy.html#subsec_solution_strategy_runtime_environment", [
          [ "Principles and patterns", "page_solution_strategy.html#subsubsec_solution_strategy_rte_principles", null ],
          [ "Software component ports", "page_solution_strategy.html#subsubsec_solution_strategy_swc_ports", null ]
        ] ]
      ] ],
      [ "Signal processing algorithms", "page_solution_strategy.html#sec_solution_strategy_algorithms", [
        [ "Estimation of attitude and vertical motion", "page_solution_strategy.html#subsec_solution_strategy_attitude_and_vertical_motion_estimation", [
          [ "Introduction", "page_solution_strategy.html#subsubsec_solution_strategy_estimation_intro", null ],
          [ "State vector", "page_solution_strategy.html#subsubsec_solution_strategy_estimation_state_vector", null ],
          [ "State error vector", "page_solution_strategy.html#subsubsec_solution_strategy_estimation_error_state_vector", null ],
          [ "Technology decisions", "page_solution_strategy.html#subsubsec_solution_strategy_estimation_decisions", [
            [ "Decoupling of magnetic heading estimation", "page_solution_strategy.html#par_solution_strategy_technology_decision_1", null ],
            [ "Single IMU signal source for every filter", "page_solution_strategy.html#par_solution_strategy_technology_decision_2", null ]
          ] ]
        ] ]
      ] ]
    ] ],
    [ "Building block view", "page_building_block_view.html", [
      [ "Level 0", "page_building_block_view.html#sec_bb_view_level_0", null ],
      [ "Level 1", "page_building_block_view.html#sec_bb_view_level_1", [
        [ "Application software", "page_building_block_view.html#subsec_asw", null ],
        [ "Basic software", "page_building_block_view.html#subsec_bsw", null ],
        [ "Runtime Environment", "page_building_block_view.html#subsec_rte", null ]
      ] ],
      [ "Level 2", "page_building_block_view.html#sec_bb_view_level_2", [
        [ "Basic software", "page_building_block_view.html#subsec_bsw_level_2", [
          [ "List of basic software components", "page_building_block_view.html#subsubsec_bsw_swc_list", null ]
        ] ],
        [ "Runtime environment", "page_building_block_view.html#subsec_rte_level_2", null ],
        [ "Application software", "page_building_block_view.html#subsec_asw_level_2", [
          [ "List of application software components", "page_building_block_view.html#subsubsec_asw_swc_list", null ]
        ] ]
      ] ],
      [ "Level 3", "page_building_block_view.html#sec_bb_view_level_3", [
        [ "Application software", "page_building_block_view.html#subsec_asw_level_3", [
          [ "UML component diagram", "page_building_block_view.html#subsubsec_asw_level_3_component_diagram", null ]
        ] ],
        [ "Basic software", "page_building_block_view.html#subsec_bsw_level_3", [
          [ "UML component diagram", "page_building_block_view.html#subsubsec_bsw_level_3_component_diagram", null ]
        ] ]
      ] ]
    ] ],
    [ "Runtime view", "page_runtime_view.html", [
      [ "BAHRS software activity diagram", "page_runtime_view.html#sec_runtime_view_bahrs_sw_activity", null ],
      [ "FreeRTOS tasks", "page_runtime_view.html#sec_runtime_view_freertos_tasks", [
        [ "Task scheduling and interactions", "page_runtime_view.html#subsec_runtime_view_scheduling_and_interaction", [
          [ "Signal chain: BAHRS Filter 1", "page_runtime_view.html#subsubsec_runtime_view_filter_chain_1", null ],
          [ "Signal chain: BAHRS Filters 2 and 3", "page_runtime_view.html#subsubsec_runtime_view_filter_chain_2_and_3", null ]
        ] ],
        [ "Internal behavior of tasks", "page_runtime_view.html#subsec_runtime_view_task_internal_behavior", [
          [ "Cyclic task (5ms interval)", "page_runtime_view.html#subsubsec_runtime_view_task_5_ms", null ],
          [ "Cyclic task (10ms interval)", "page_runtime_view.html#subsubsec_runtime_view_task_10_ms", null ]
        ] ]
      ] ],
      [ "Software components", "page_runtime_view.html#sec_runtime_view_internal_behavior_swc", [
        [ "Internal behavior of IMU monitor SWC", "page_runtime_view.html#subsec_runtime_view_imu_monitor_activity", null ],
        [ "Internal behavior of Barometer monitor SWC", "page_runtime_view.html#subsec_runtime_view_barometer_monitor_activity", null ],
        [ "Internal behavior of Vertical channel monitor SWC", "page_runtime_view.html#subsec_runtime_view_vertical_channel_monitor_activity", null ],
        [ "behavior of Attitude monitor SWC", "page_runtime_view.html#subsec_runtime_view_attitude_monitor_activityInternal", null ],
        [ "FDI activity for an arbitrary scalar signal", "page_runtime_view.html#subsec_runtime_view_scalar_signal_fdi_activity", null ]
      ] ]
    ] ],
    [ "Deployment view", "page_deployment_view.html", null ],
    [ "Cross-cutting concepts", "page_cross_cutting_concerns.html", null ],
    [ "Design decisions", "page_design_decisions.html", [
      [ "Runtime environment", "page_design_decisions.html#sec_design_decisions_rte", [
        [ "Port access protection", "page_design_decisions.html#subsec_design_decisions_port_protection", null ],
        [ "Software component port data types", "page_design_decisions.html#subsec_design_decisions_port_data_types", null ],
        [ "Auto-generation of runtime environment", "page_design_decisions.html#subsec_design_decisions_rte_autogeneration", null ]
      ] ]
    ] ],
    [ "Quality requirements", "page_quality_requirements.html", null ],
    [ "Risks and technical debt", "page_risks_and_tech_debt.html", null ],
    [ "Glossary", "page_glossary.html", [
      [ "Abbreviations", "page_glossary.html#sec_glossary_abbreviations", [
        [ "BAHRS", "page_glossary.html#BAHRS", null ],
        [ "SWC", "page_glossary.html#SWC", null ],
        [ "BSW", "page_glossary.html#BSW", null ],
        [ "ASW", "page_glossary.html#ASW", null ],
        [ "FDI", "page_glossary.html#FDI", null ]
      ] ],
      [ "Terms and definitions", "page_glossary.html#sec_glossary_definitions", null ]
    ] ]
];