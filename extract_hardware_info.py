import re

# Read the extracted PDF content
with open('pdf_content.txt', 'r', encoding='utf-8') as f:
    content = f.read()

# Remove excessive whitespace and join words properly
content = re.sub(r'\n+', ' ', content)
content = re.sub(r'\s+', ' ', content)

# Find hardware-related sections
hardware_keywords = [
    'Raspberry Pi 4',
    'Raspberry Pi Pico',
    'ESP32',
    'Hokuyo',
    'LiDAR',
    'bateria',
    'battery',
    '12V',
    '7Ah',
    'motor',
    'driver',
    'sensor',
    'tablet',
    'LED',
    'roteador',
    'router',
    'conversor',
    'MDF',
    'dimensões',
    'Model B',
    'componentes principais',
    'Componentes Principais'
]

# Extract sections around hardware keywords
hardware_info = {}
for keyword in hardware_keywords:
    pattern = rf'.{{0,500}}{keyword}.{{0,500}}'
    matches = re.findall(pattern, content, re.IGNORECASE)
    if matches:
        hardware_info[keyword] = matches

# Print organized hardware information
print("=" * 80)
print("INFORMAÇÕES DE HARDWARE DO ROBÔ DC - 1ª GERAÇÃO")
print("=" * 80)
print()

for keyword, matches in hardware_info.items():
    print(f"\n{'='*80}")
    print(f"KEYWORD: {keyword}")
    print(f"{'='*80}")
    for i, match in enumerate(matches[:3], 1):  # Show top 3 matches
        print(f"\nMatch {i}:")
        print(match)
        print("-" * 80)
