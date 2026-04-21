import os

# Define the file extensions to include
extensions = {'.h', '.cpp', '.txt', '.html','.ts', '.tsx', '.yml', '.java', '.json', '.html', '.js', '.jsx', '.kts', '.css', '.properties', '.py', '.glsl',}
exclude_dirs = {
    'node_modules',
    'build',
    'lib',
    'lib64',
    'bin',
    'dist',
    'assets',
    'ui_old',
    '.git',
    'out',
    '.gradle',
    '.idea',
    '.scripts',
    '.vite',
    'coverage',
    'lcov-report',
    '.pio',
    '.pio-core',
    '.vscode'
}
exclude_files = {
    'package-lock.json',
    '.gitlab-ci.yml',
    'lint-report.json'
}

output_file = 'all_files_concatenated.txt'

with open(output_file, 'w', encoding='utf-8') as out:
    for root, dirs, files in os.walk('.'):
        dirs[:] = [d for d in dirs if d not in exclude_dirs]

        for name in files:
            ext = os.path.splitext(name)[1]
            if ext in extensions and name not in exclude_files:
                filepath = os.path.join(root, name)
                relpath = os.path.relpath(filepath, '.')
                out.write(f"/// === FILE: {relpath} ===\n")
                try:
                    with open(filepath, 'r', encoding='utf-8') as f:
                        out.write(f.read())
                except Exception as e:
                    out.write(f"// ERROR: Could not read file: {e}\n")
                out.write("\n\n")
print(f"Created {output_file} in the current directory.")


