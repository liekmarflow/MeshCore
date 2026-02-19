import fitz
import sys

doc = fitz.open(r'c:\Users\wolfr\Documents\VS-Code-Projects\extern\MeshCore\variants\inhero_mr2\docs\NCP15XH103F03RC.pdf')
with open(r'c:\Users\wolfr\Documents\VS-Code-Projects\extern\MeshCore\variants\inhero_mr2\docs\NCP15XH103F03RC.txt', 'w', encoding='utf-8') as f:
    for i, page in enumerate(doc):
        f.write(f'--- Page {i+1} ---\n')
        f.write(page.get_text())
        f.write('\n')
print("Done - extracted to NCP15XH103F03RC.txt")
