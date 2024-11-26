Import("env")
import os
import re
import subprocess
import datetime

# 以下の情報を出力する
# - ビルド日時
# - コミットハッシュ
# - git patch

git_ver = subprocess.run(['git', 'describe', '--tags', '--always', '--dirty'], stdout=subprocess.PIPE).stdout.decode('utf-8').strip()


version_info_h = """
#pragma once

static const char * BUILD_DATE =  "{}";
static const char * GIT_REVISION = "{}";
""".format(datetime.datetime.now(), git_ver )


def output_version_info():
    print("Outputting version information")
    filename = os.path.join(env['PROJECT_DIR'], 'src', 'version.h')

    with open(filename, "w+") as file:
        file.write(version_info_h)


output_version_info()