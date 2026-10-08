from glob import glob

from setuptools import setup

package_name = 'cart_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/routes', glob('routes/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neel',
    maintainer_email='neelnaik2005@gmail.com',
    description='Driving stack for the BITSAuto cart: throttle-only speed hold, safety monitor, local planner.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
