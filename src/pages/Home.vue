<script setup lang="ts">
import Splitter from 'primevue/splitter';
import SplitterPanel from 'primevue/splitterpanel';
import ScrollPanel from 'primevue/scrollpanel';
import SplitButton from 'primevue/splitbutton';
import Divider from 'primevue/divider';
import Card from 'primevue/card';
import Fieldset from 'primevue/fieldset';
import Panel from 'primevue/panel';
import Tag from 'primevue/tag';
</script>
<template>
  <div>
    <Splitter style="width: 100%; height: calc(100vh - var(--nav-bar-size))" class="mb-8">
      <SplitterPanel :size="75">
        <ScrollPanel style="width: 100%; height: calc(100vh - var(--nav-bar-size))">
          <h1>Create a simple node</h1>
          <Panel header="Creating a Package" toggleable>
            <div>
              <ol>
                <li>
                  Make sure you are inside your Docker container. See
                  <a href="./getting_started.md">getting_started.md</a>
                  if you do not know how
                </li>
                <li>
                  Change your directory to <Tag value="/src"></Tag>
                  First determine what directory you are currently in using
                  <Tag value="pwd"></Tag> (print working directory).
                </li>
                <li>
                  Type <Tag value="ls"></Tag> (list) to determine what files and directories are in
                  your current working directory. Type
                  <Tag value="cd <name of folder>"></Tag> (change directory) in order to move into a
                  directory, aka a folder. Do this until you reach
                  <Tag value="/urc-2023/src"></Tag>.
                </li>

                Create a new Python node package
                <li>
                  Run the following command
                  <Tag value="ros2 pkg create --build-type ament_python <package_name>"></Tag>
                </li>
                I ran
                <Tag value="ros2 pkg create --build-type ament_python example_node"></Tag>
              </ol>
            </div>
          </Panel>
          <Panel header="Writing a Basic Node to Say 'Hello World'" toggleable collapsed>
            <div>
              <ol>
                <li>
                  You will need to import the ROS Python library, create a class representing your
                  node, create an entry point, and write code to "gracefully-ish" shutdown the node
                  when we press <Tag value="ctrl-c"></Tag>. Make sure to read the
                  <a href="./formatting.md">formatting.md</a> document to ensure you are writing
                  good readable code.
                </li>
                <li>
                  An important detail to remember is the name of your node:
                  <Tag value="super().__init__('this_is_the_name_of_the_node')"></Tag>
                </li>
              </ol>
              <pre>
      <code>
import sys
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lib.color_codes import ColorCodes, colorStr

class ExampleNode(Node):
    def __init__(self) -> None:
        super().__init__("my_example_node")
        self.get_logger().info(colorStr("Launching example_node node", ColorCodes.BLUE_OK))

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        example = ExampleNode()
        rclpy.spin(example)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        example.get_logger().info(colorStr("Shutting down example_node node", ColorCodes.BLUE_OK))
        example.destroy_node()
        sys.exit(0)
      </code>
    </pre>
            </div>
          </Panel>

          <Panel header="Adding to the List of Launch-able Nodes" toggleable collapsed>
            <div>
              <ol>
                <li>
                  Go back to your <Tag value="setup.py"></Tag> file. Look at the
                  <Tag value="console_scripts"></Tag>, it should look like this right now:
                  <Tag value="'console_scripts': [],"></Tag>
                </li>
                <li>
                  Add an entry that points towards your <Tag value="main()"></Tag> function found in
                  your Python file that you added. The syntax is the following:
                  <Tag
                    value="'console_scripts': ['<name of executable> = <name of package>.<name of the python file that has the main() function>:main'],"
                  ></Tag>
                </li>
                <li>
                  For me, it would be this:
                  <Tag
                    value="'console_scripts': ['myExampleNode = example_node.example:main'],"
                  ></Tag>
                </li>
                <li>
                  Open the file <Tag value="robot.launch.py"></Tag> under the folder
                  <Tag value="/src/viator_launch"></Tag>. Create a new variable to store your node.
                  The <Tag value="package"></Tag> parameter should be the same name as your node
                  package name. The <Tag value="executable"></Tag> parameter should be the variable
                  name you chose in the <Tag value="setup.py"></Tag> file. The
                  <Tag value="name"></Tag> is the name of your node you wrote in the code in the
                  <Tag value="super().__init__()"></Tag> method.
                </li>
                <li>
                  For me, I wrote the following:
                  <Tag
                    value="example_node = Node(package='example_node', executable='myExampleNode', name='my_example_node')"
                  ></Tag>
                </li>
                <li>
                  In the same <Tag value="robot.launch.py"></Tag> file, add it to the array in the
                  <Tag value="generate_launch_description()"></Tag> function:
                  <pre>
          <code>
def generate_launch_description() -> launch.LaunchDescription:
    return launch.LaunchDescription(
        [can_moteus_node, drivebase_node, mission_control_updater_node, arm_node, example_node] # <- I added it to the end of the array
)
          </code>
        </pre>
                </li>
                <li>
                  In the <Tag value="/src/viator_launch"></Tag> folder, open up the
                  <Tag value="package.xml"></Tag> file. Add a new
                  <Tag value="<exec_depend>"></Tag> entry to tell that your package should be
                  included during build. The content is the name of your node package.
                </li>
                <li>
                  For me, it is this:
                  <Tag value="<exec_depend>example_node</exec_depend>"></Tag>
                </li>
              </ol>
            </div>
          </Panel>

          <Panel header="Build and Execute Code" toggleable collapsed>
            <div>
              <ol>
                <li>
                  All while in the Docker container, execute the shell scripts
                  <Tag value="./build.sh"></Tag> and <Tag value="./launch.sh"></Tag> in the command
                  line. You might have to delete the <Tag value="/build"></Tag> and
                  <Tag value="/install"></Tag> folders.
                </li>
                <li>
                  Congrats, you have made a new node! Yippie! Further examples of how to use ROS
                  timers, subscribers, and publishers are found in the
                  <Tag value="/src/example_node/example_node"></Tag> node; go ahead, open it up, and
                  read the code.
                </li>
              </ol>
            </div>
          </Panel>
        </ScrollPanel>
      </SplitterPanel>
      <SplitterPanel :size="15" :minSize="10"> Panel 2 </SplitterPanel>
    </Splitter>
  </div>
</template>

<style lang="scss" scoped></style>
