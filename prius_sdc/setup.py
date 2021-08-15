from setuptools import setup
import os
from glob import glob

from setuptools import find_packages

# packages=find_packages(exclude=['Detection'])


package_name = 'prius_sdc'
detection_L_module ="prius_sdc/Detection/Lanes"
detection_S_module ="prius_sdc/Detection/Signs"
config_module = "prius_sdc/config" 
data_module ="prius_sdc/data"
Control_module ="prius_sdc/Control"


setup(
    name=package_name,
    version='0.0.0',
    packages=['prius_sdc',Control_module,data_module,detection_S_module,detection_L_module,config_module],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luqman',
    maintainer_email='noshluk2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'sdc = prius_sdc.Luqman_Stupid_Class:main',

        ],
    },
)
