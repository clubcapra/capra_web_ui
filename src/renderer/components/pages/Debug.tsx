import React, { FC, useCallback, useEffect, useState } from 'react';
import { styled } from '@/renderer/globalStyles/styled';
import { SectionTitle } from '@/renderer/components/pages/Config/styles';
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types';
import { ChangeEvent } from 'react';
import { Select } from '@/renderer/components/common/Select';
import { useRosSubscribeNoData } from '@/renderer/hooks/useRosSubscribe';
import { flippersViewToggleSlice } from '@/renderer/store/modules/flippersViewToggle';
import { useDispatch } from 'react-redux';

const maxLine = 200;
const topic: TopicOptions<Line> = {
  name: '/rosout',
  messageType: 'rosgraph_msgs/Log',
};
const nofilter = 'All';

type Line = {
  msg: string;
  topics: Array<string>;
  level: Severity;
};

enum Severity {
  ALL = 0,
  DEBUG = 1,
  INFO = 2,
  WARN = 4,
  ERROR = 8,
  FATAL = 16,
}

const initialLine: Line = {
  msg: '',
  topics: [],
  level: Severity.ALL,
};

const SaveText: FC<{ text: Array<Line> }> = ({ text }) => {
  const lines = new Array<string>();
  text.map((line) => {
    lines.push(line.msg);
  });
  const data = new Blob([lines.join('\n')], { type: 'text/plain' });

  const downloadLink = URL.createObjectURL(data);
  const fileName = `log-${new Date().toUTCString()}.txt`;
  return (
    <a href={downloadLink} download={fileName}>
      Save as text
    </a>
  );
};

interface ActionsAreaProps {
  topics: Array<string>;
  selectedTopic: string;
  setSelectedTopic: (topic: string) => void;
  selectedSeverity: number;
  setSelectedSeverity: (severity: number) => void;
  allLines: Array<Line>;
  setConsole: (debugConsole: Array<Line>) => void;
  errorTopics: Array<string>;
}

const ActionsArea: FC<ActionsAreaProps> = ({
  topics,
  selectedTopic,
  setSelectedTopic,
  selectedSeverity,
  setSelectedSeverity,
  allLines,
  setConsole,
  errorTopics,
}) => {
  const filterByTopic = (e: ChangeEvent<HTMLSelectElement>) => {
    const topic = topics[e.target.selectedIndex];
    setSelectedTopic(topic);

    filterLines(topic, selectedSeverity);
  };

  const filterBySeverity = (e: ChangeEvent<HTMLSelectElement>) => {
    const severity = Number.parseInt(
      Object.keys(Severity)[e.target.selectedIndex]
    );
    setSelectedSeverity(severity);

    filterLines(selectedTopic, severity);
  };

  const filterLines = (topic: string, severity: number) => {
    const filteredLines = new Array<Line>();
    allLines.map((line) => {
      if (
        (line.topics.includes(topic) || topic == nofilter) &&
        (line.level == severity || severity == Severity.ALL)
      ) {
        filteredLines.push(line);
      }
    });

    setConsole(filteredLines);
  };

  return (
    <ActionsWrapper>
      Filter by topic :
      <Select
        onChange={filterByTopic}
        value={selectedTopic}
        options={topics.map((topic) => ({
          key: topic.toString(),
          value: topic.toString(),
        }))}
      />
      <br />
      Filter by severity :
      <Select
        onChange={filterBySeverity}
        value={Severity[selectedSeverity]}
        options={Object.keys(Severity)
          .filter((k) => typeof Severity[k as never] === 'number')
          .map((severity) => ({
            key: severity,
            value: severity.toString(),
          }))}
      />
      <br />
      <SaveText text={allLines} />
      <br />
      <br />
      Topics that emitted an error :
      {errorTopics.map((topic) => {
        return <p key={topic}> {topic} </p>;
      })}
    </ActionsWrapper>
  );
};

interface ConsoleAreaProps {
  debugConsole: Array<Line>;
}
const ConsoleArea: FC<ConsoleAreaProps> = ({ debugConsole }) => {
  return (
    <ConsoleWrapper>
      <SectionTitle>Console</SectionTitle>
      <DarkContainer>
        {debugConsole.map((line) => {
          return <p key={line.msg}> {line.msg}</p>;
        })}
      </DarkContainer>
    </ConsoleWrapper>
  );
};

export const DarkContainer = styled.div`
  background-color: ${({ theme }) => theme.colors.darkerBackground};
`;

const DebugConfigWrapper = styled.div`
  width: 100%;
  height: 100%;
  display: grid;
  grid-template: 'm r';
  grid-template-columns: 1fr 5fr;
  height: 100%;
  max-height: 100%;
`;

const ActionsWrapper = styled.div`
  grid-area: m;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  margin: 0;
  min-height: 100%;
  padding: 2%;
`;

const ConsoleWrapper = styled.div`
  grid-area: r;
  padding: 1%;
  overflow: auto;
`;

export const Debug: FC = () => {
  const [newMessage, setMessage] = useState(initialLine);
  const [debugConsole, setConsole] = useState(Array<Line>());
  const [allLines, setLines] = useState(Array<Line>());
  const [topics, setTopics] = useState(Array<string>(nofilter));
  const [selectedTopic, setSelectedTopic] = useState(nofilter);
  const [selectedSeverity, setSelectedSeverity] = useState(Severity.ALL);
  const [errorTopics, setErrorTopics] = useState(Array<string>());
  const dispatch = useDispatch();

  useRosSubscribeNoData(
    topic,
    useCallback((message) => {
      setMessage(message);
    }, [])
  );

  // New message received.
  useEffect(() => {
    const lines = [...allLines];
    const filteredLines = [...debugConsole];
    const topicList = [...topics];
    const errorList = [...errorTopics];
    const message = newMessage;

    message.topics.map((topic) => {
      if (!topicList.includes(topic)) {
        topicList.push(topic);
      }

      if (
        (message.level == Severity.ERROR || message.level == Severity.FATAL) &&
        !errorList.includes(topic)
      ) {
        errorList.push(topic);
      }
    });

    const line: Line = {
      msg: message.msg,
      topics: message.topics,
      level: message.level,
    };
    lines.push(line);
    if (
      (message.topics.includes(selectedTopic) || selectedTopic == nofilter) &&
      (message.level == selectedSeverity || selectedSeverity == Severity.ALL)
    ) {
      filteredLines.push(line);
    }

    if (lines.length > maxLine) {
      lines.splice(0, lines.length - maxLine);
    }
    if (filteredLines.length > maxLine) {
      filteredLines.splice(0, lines.length - maxLine);
    }

    dispatch(flippersViewToggleSlice.actions.setNotVisible());

    setLines(lines);
    setTopics(topicList);
    setErrorTopics(errorList);
    setConsole(filteredLines);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [newMessage, dispatch]);

  return (
    <DebugConfigWrapper>
      <ActionsArea
        topics={topics}
        selectedTopic={selectedTopic}
        setSelectedTopic={setSelectedTopic}
        selectedSeverity={selectedSeverity}
        setSelectedSeverity={setSelectedSeverity}
        allLines={allLines}
        setConsole={setConsole}
        errorTopics={errorTopics}
      />

      <ConsoleArea debugConsole={debugConsole} />
    </DebugConfigWrapper>
  );
};
