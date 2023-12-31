from setuptools import find_packages, setup
from glob import glob
import os
from setuptools import find_packages

package_name = 'sign_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'), glob(os.path.join('data', '*.png'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asphodel',
    maintainer_email='asphodel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sign_detection = sign_detector.sign_detector:main',
        ],
    },
)
