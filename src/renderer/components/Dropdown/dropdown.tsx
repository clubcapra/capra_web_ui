import React from 'react';


interface robot {
    name: string
}

function BasicExample() {

    const robot: robot[] = [
        {
            name:"rove",
        },
        {
            name:"markhor",
        },
    ]
  return (
    <div>
    <select>
        {robot.map((robot) => (
            <option value={robot.name}>{robot.name}</option>
          ))}
    </select>
  </div>
  );
}

export default BasicExample;