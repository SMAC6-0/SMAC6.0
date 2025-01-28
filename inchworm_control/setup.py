from setuptools import setup, find_packages

sub_package1 = 'inchworm_control'
sub_package2 = 'block_simulation'

setup(
    name=sub_package1,
    version='0.0.0',
    # packages=[sub_package1],

    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + sub_package1]),
        ('share/' + sub_package1, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = inchworm_control.motor_controller:main'
            'step_publisher = inchworm_control.step_publisher:main'            
        ],
    },
    
    name=sub_package2,
    version='0.0.0',
    # packages=[sub_package1],

    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + sub_package2]),
        ('share/' + sub_package1, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = inchworm_control.motor_controller:main'
            'step_publisher = inchworm_control.step_publisher:main'            
        ],
    },
)

