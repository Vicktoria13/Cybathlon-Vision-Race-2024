from setuptools import setup

package_name = 'navigation_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='polytech',
    maintainer_email='victoria-nguyen@orange.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dijkstra_node = navigation_py.dijkstra_node:main',
            'direction_node = navigation_py.direction:main',
            'direction_node_ss_bool = navigation_py.direction_ss_bool:main'
        ],
    },
)
