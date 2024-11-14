import ROSLIB from 'roslib';
import { useRoslibStore } from '@/store/useRoslib';
import type { TopicType, TopicTypeMap } from './rosTypes';

/**
 * A publisher that can send out messages to a certain topic.
 */
export interface Publisher<T extends TopicType> {
  /**
   * Publishes the given data to the configured topic.
   * @param data - is the data to publish.
   * @param options - are the options to use during this publish.
   * @param options.isDebugging - should debug output be displayed?
   */
  publish: (data: TopicTypeMap[T], options?: { isDebugging?: boolean }) => void;
}

/**
 * Creates a publisher that can send out messages to a certain topic.
 * @param options.topicName - should start with '/' along with topic name
 * @param options.topicType - TopicType Ros Message Type
 * @param options.isDebugging - optional prints to console for debugging
 */
export default function createPublisher<T extends TopicType>(options: {
  topicName: string;
  topicType: T;
}): Publisher<T> {
  const { topicName, topicType } = options;
  const ros = useRoslibStore().ros;
  const topic = new ROSLIB.Topic<TopicTypeMap[T]>({
    ros,
    name: topicName,
    messageType: topicType,
    compression: 'cbor',
  });

  const publish: Publisher<T>['publish'] = (data, options) => {
    const { isDebugging } = options || {};
    topic.publish(data);
    if (isDebugging) {
      console.log(`[${topicName}] Publishing:`, data);
    }
  };

  // TODO: Publisher that runs every interval has passed
  // publishTimer(options:{callback? : () => void, interval : number, isDebugging? : boolean })
  // returns Object so if needed to add more methods, cane easily do and it doesn't create topic object everytime
  return { publish };
}
