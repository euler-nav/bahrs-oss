# Software documentation

The architecture documentation is [published online](https://euler-nav.github.io/bahrs-arch-docs/). Its source pages are in [doc_pages](doc_pages/). Practical how-tos, such as reprogramming a device and the debug connector pinout, are in the [knowledge base](KNOWLEDGE_BASE.md).

To generate the BAHRS rev. 2 or rev. 3 documentation locally on Windows, install Java and make sure `java` is on `PATH`. Doxygen and PlantUML are supplied in the repository.

From this directory, run the matching pair of scripts:

- Rev. 2: `generate_documentation_v2.bat`, then `open_documentation_v2.bat`
- Rev. 3: `generate_documentation_v3.bat`, then `open_documentation_v3.bat`

The generated site is written to `output_V2\html` or `output_V3\html`.
