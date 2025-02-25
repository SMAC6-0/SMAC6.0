from setuptools import setup, find_packages

setup(
    name='inchworm_project',
    version='0.1.0',
    packages=find_packages(),  # Automatically find all packages
    install_requires=[],  # Add dependencies here if needed
    entry_points={
        'console_scripts': [
            'motor_controller = inchworm_control.motor_controller:main',
            'step_publisher = inchworm_control.step_publisher:main',
        ],
    },
)
