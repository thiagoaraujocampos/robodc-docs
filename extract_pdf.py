import PyPDF2
import sys

pdf_path = r'c:\Users\Thiago\Downloads\tcc\Descrição de Arquitetura - RoboDC.pdf'
output_file = r'c:\Users\Thiago\ufscar\robodc-docs\pdf_content.txt'

try:
    with open(pdf_path, 'rb') as file:
        reader = PyPDF2.PdfReader(file)
        
        with open(output_file, 'w', encoding='utf-8') as out:
            out.write(f"Total pages: {len(reader.pages)}\n\n")
            
            for i, page in enumerate(reader.pages):
                out.write(f"=== PAGE {i+1} ===\n")
                text = page.extract_text()
                out.write(text)
                out.write("\n" + "="*80 + "\n\n")
        
        print(f"Content extracted successfully to {output_file}")
except Exception as e:
    print(f"Error: {e}")
    sys.exit(1)
