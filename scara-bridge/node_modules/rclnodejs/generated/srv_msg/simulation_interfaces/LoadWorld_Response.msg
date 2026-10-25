

uint8 UNSUPPORTED_FORMAT = 101              # Format for uri or resource string is unsupported.
uint8 NO_RESOURCE = 102                     # Both uri and resource string are empty.
uint8 RESOURCE_PARSE_ERROR = 103            # Resource file or string failed to parse.
uint8 MISSING_ASSETS = 104                  # At least one of resource assets (such as meshes) was not found.
uint8 UNSUPPORTED_ASSETS = 105              # At least one of resource assets (such as meshes) is not supported.
uint8 UNSUPPORTED_ELEMENTS = 106            # At least one of world definition elements such as format tags is unsupported.


Result result                               # Standard result message

WorldResource world                         # Information about the loaded world. Only valid if result is RESULT_OK.
