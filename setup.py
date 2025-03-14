from setuptools import setup
from glob import glob

package_name = 'fbot_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gpsr.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/boris.urdf']),
        ('share/' + package_name + '/materials/textures', ['materials/textures/parede.png']),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cris',
    maintainer_email='cris.lima.froes@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpsr = fbot_simulation.gpsr:main'
        ],
    },
)
