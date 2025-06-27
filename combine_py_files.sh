import os

output_file = "combined_real_python_code.py"

def is_probably_notebook(file_path):
    with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
        first_char = f.read(1)
        return first_char == "{"

with open(output_file, "w", encoding="utf-8") as out_f:
    for root, dirs, files in os.walk("."):
        if ".git" in root:
            continue
        for file in sorted(files):
            if file.endswith(".py"):
                path = os.path.join(root, file)
                if is_probably_notebook(path):
                    print(f"Skipping notebook-like file: {path}")
                    continue
                print(f"Adding file: {path}")
                out_f.write(f"\n\n# ======= {path} =======\n\n")
                with open(path, "r", encoding="utf-8", errors="ignore") as in_f:
                    out_f.write(in_f.read())
                out_f.write("\n")

print(f"✅ Combined pure Python code saved to {output_file}")
