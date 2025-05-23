from setuptools import setup

package_name = 'system_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='System-wide launch configuration',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)

