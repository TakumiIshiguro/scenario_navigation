# scenario_navigation  

### Requirement  

Mecab
```
pip3 install mecab-python3
```
```
git clone --depth 1 https://github.com/neologd/mecab-ipadic-neologd.git
cd mecab-ipadic-neologd
./bin/install-mecab-ipadic-neologd -n
```
### Execute  

- Simulator (without vison)  
1) Launch the simulator  


2) Move robot to initial position  

3) Select a scenario in "navigation.launch"  
`<param name="scenario_path" value="$(find scenario_navigation)/config/Scenarios/scenario01.txt" />`

4) Execute the scenario navigation  with target direction
`roslaunch scenario_navigation navigation_cmd.launch`


- Real Robot (with vison)  
1) Execute the scenario navigation with vision  
`roslaunch scenario_navigation navigation_with_yolo.launch`  
