from setuptools import setup, find_packages

setup(
    name='inchworm_project',  # Overall project name
    version='0.0.1',
    packages=find_packages(exclude=['tests']),  # Finds all sub-packages
    install_requires=['setuptools'],  # Dependencies

    # Optional metadata
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],

    # Entry points for any scripts within your packages
    entry_points={
        'console_scripts': [
            'motor_controller = inchworm_control.motor_controller:main',
            'step_publisher = inchworm_control.step_publisher:main',
        ],
    },
)
