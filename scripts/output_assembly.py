Import("env")

def after_build(source, target, env):
    env.Execute(env.subst("arm-none-eabi-objdump -d {0} > {1}.lst".format(str(target[0]), str(target[0]))))

env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", after_build)