from setuptools import find_packages, setup

package_name = 'obstacle_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'transforms3d'],
    zip_safe=True,
    maintainer='nicolas-lauzon',
    maintainer_email='nicolas_lauzon@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obstacle_detection = obstacle_detection.obstacle_detection:main'
        ],
    },
)
