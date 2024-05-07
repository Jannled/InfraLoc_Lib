Import("env")

blacklist = [
    "inih/examples",
    "inih/fuzzing",
    "inih/tests"
]

def skip_from_build(node):
    """
    `node.name` - a name of File System Node
    `node.get_path()` - a relative path
    `node.get_abspath()` - an absolute path
     to ignore file from a build process, just return None
    """
    for b in blacklist:

        if b in node.get_path():
            # Return None for exclude
            print(f'[BLACKLIST] Skipping "{node.get_path()}"')
            return None

    return node

# Register callback
env.AddBuildMiddleware(skip_from_build, "*")