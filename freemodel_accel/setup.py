from setuptools import find_packages, setup

package_name = 'freemodel_accel'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='basanta',
    maintainer_email='thebasantaadhikari@gmail.com',
    description='Updated freemodel branch using acceleration-based control.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fm_follower = freemodel_accel.fm_follower:main',
            'fm_leader = freemodel_accel.fm_leader:main',
            'fm_pipeline = freemodel_accel.fm_pipeline:main',
            'fm_offline_spsa = freemodel_accel.fm_offline_spsa:main',
            'fm_demo = freemodel_accel.fm_demo:main',
        ],
    },
)
