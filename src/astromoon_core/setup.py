from setuptools import find_packages, setup
from pathlib import Path

package_name = 'astromoon_core'

def package_files(directory: str):
    entries = []
    base = Path('share') / package_name
    for p in Path(directory).rglob('*'):
        if p.is_file():
            # recrée l'arborescence dans share/<pkg>/
            dest = base / p.parent
            entries.append((str(dest), [str(p)]))
    return entries

data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    (f'share/{package_name}', ['package.xml']),
]

# Installe récursivement les assets (portable docker/host)
# Ajoute/retire des dossiers ici selon ton repo
for d in ['launch', 'worlds', 'urdf', 'models', 'meshes']:
    if Path(d).exists():
        data_files += package_files(d)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emilien',
    maintainer_email='emilienghazal@gmail.com',
    description='astromoon rover',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
