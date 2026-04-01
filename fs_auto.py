from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

def after_upload(source, target, env):
    # Build FS image
    env.Execute("pio run -t buildfs -e " + env['PIOENV'])
    # Upload FS image
    env.Execute("pio run -t uploadfs -e " + env['PIOENV'])

# When the normal 'upload' target finishes, also build+upload FS
env.AddPostAction("upload", after_upload)
