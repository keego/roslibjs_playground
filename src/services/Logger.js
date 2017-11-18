/* eslint-disable no-console */
export default function logger(tag, options = {}) {
  return {
    debug(...msgs) {
      console.log(
        `%c[${tag}]`,
        `color: ${options.debugColor || 'rgba(100, 150, 100, 1)'}`,
        ...msgs,
      )
    },
    todo(...msgs) {
      console.log(
        '%c[TODO]',
        `color: ${options.todoColor || 'rgba(150, 200, 150, 1)'}`,
        ...msgs,
        `(in ${tag})`,
      )
    },
    info(...msgs) {
      console.info(
        `%c[${tag}]`,
        `color: ${options.infoColor || 'rgba(0, 150, 200, 1)'}`,
        ...msgs,
      )
    },
    warn(...msgs) {
      console.warn(
        `%c[${tag}]`,
        `color: ${options.warnColor || 'rgba(180, 180, 100, 1)'}`,
        ...msgs,
      )
    },
    error(...msgs) {
      console.error(
        `%c[${tag}]`,
        `color: ${options.errorColor || 'rgba(180, 100, 100, 1)'}`,
        'Error:',
        ...msgs,
      )
    },
  }
}
