import ROSLIB from 'roslib';
import { useRoslibStore } from '../../store/useRoslib';
import type { StdMsg, TopicType, TopicTypeMap } from './rosTypes';

export type Publisher<T extends TopicType> = {
  publish: (options: { data: TopicTypeMap[T]; isDebugging?: boolean }) => void;
};

/**
 * Creates Generic Publisher to interact with Ros.
 * @param options.topicName should start with '/' along with topic name
 * @param options.topicType TopicType Ros Message Type
 * @param options.isDebugging? optional prints to console for debugging
 * @returns {Object} Publisher
 * @returns Object.publish given data
 */
export default function createPublisher<T extends TopicType>(options: {
  topicName: string;
  topicType: T;
}): Publisher<T> {
  const { topicName, topicType } = options;
  const ros = useRoslibStore().ros;
  const topic = new ROSLIB.Topic<StdMsg<TopicTypeMap[T]>>({
    ros,
    name: topicName,
    messageType: topicType,
    compression: 'cbor',
  });
  /**
   * Publish given data
   * @param data publishes data
   */
  const publish = (options: { data: TopicTypeMap[T]; isDebugging?: boolean }) => {
    const { data, isDebugging } = options;
    const message: StdMsg<TopicTypeMap[T]> = { data };
    topic.publish(message);
    if (isDebugging) {
      console.log(`[${topicName}] Publishing: ${data}`);
    }
  };
  // TODO: Publisher that runs every interval has passed
  // publishTimer(options:{callback? : () => void, interval : number, isDebugging? : boolean })
  // returns Object so if needed to add more methods, cane easily do and it doesn't create topic object everytime
  return { publish };
}
