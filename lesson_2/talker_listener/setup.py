from setuptools import find_packages, setup
import os
import glob
package_name = 'talker_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/im_pub.launch.py']),
        ('share/' + package_name + '/data', glob.glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='debaumann@student.ethz.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'im_pub = talker_listener.im_pub:main',
            'talker = talker_listener.republisher:main',
        ],
    },
)
