Import("env")
import os
import re

def check_duplicates():
    filename = os.path.join(env['PROJECT_DIR'], 'src', 'calibration.cpp')

    with open(filename, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    parameter_lines = [re.search(r'parameter\s+\w+\s*\([^)]*"([^"]*)"', line) for line in lines if 'parameter ' in line]

    seen = set()
    errors_found = False
    for match in parameter_lines:
        if match and match.group(1) in seen:
            print(f"\033[31mパラメータの説明が重複しています。この説明は重複してはいけません。右記を見直してください。: {match.group(1)}\033[0m")
            errors_found = True
        if match:
            seen.add(match.group(1))

    if errors_found:
        print("\033[31mError:キャリブレーションパラメータの説明(Description)が重複しています。\033[0m")
        env.Exit(1)
        # env.Execute("pio run -t clean")
    else:
        print("No duplicate parameter descriptions found.")

check_duplicates()